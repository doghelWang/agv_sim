//! Lightweight RESTful layer (port of common/rest.py): router, HTTP/1.1 keep-alive server, RFC 9457 errors,
//! self-describing index + OpenAPI 3.1, CORS, raw stream mounts, and a persistent-connection client.
use regex::Regex;
use serde_json::{json, Map, Value};
use std::collections::HashMap;
use std::io::{BufRead, BufReader, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::Arc;

pub const API_VERSION: &str = "1.0.0";
pub const API_DOC: &str = "https://github.com/doghelWang/agv_sim/blob/main/docs/API.md";

#[derive(Debug, Clone)]
pub struct ApiError { pub status: u16, pub message: String, pub code: String }
impl ApiError {
    pub fn new(status: u16, message: impl Into<String>) -> ApiError { ApiError { status, message: message.into(), code: "error".into() } }
    pub fn coded(status: u16, message: impl Into<String>, code: &str) -> ApiError { ApiError { status, message: message.into(), code: code.into() } }
}
impl std::fmt::Display for ApiError { fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result { write!(f, "{} {}", self.status, self.message) } }

pub fn phrase(status: u16) -> &'static str {
    match status {
        200 => "OK", 201 => "Created", 202 => "Accepted", 204 => "No Content", 206 => "Partial Content", 301 => "Moved Permanently", 302 => "Found", 304 => "Not Modified",
        400 => "Bad Request", 401 => "Unauthorized", 403 => "Forbidden", 404 => "Not Found", 405 => "Method Not Allowed", 408 => "Request Timeout", 409 => "Conflict",
        410 => "Gone", 413 => "Content Too Large", 415 => "Unsupported Media Type", 422 => "Unprocessable Content", 423 => "Locked", 428 => "Precondition Required", 429 => "Too Many Requests",
        500 => "Internal Server Error", 501 => "Not Implemented", 502 => "Bad Gateway", 503 => "Service Unavailable", 504 => "Gateway Timeout", _ => "Error",
    }
}

pub struct Response { pub status: u16, pub ctype: String, pub headers: Vec<(String, String)>, pub body: Vec<u8> }
impl Response {
    pub fn json(v: &Value) -> Response { Response { status: 200, ctype: "application/json; charset=utf-8".into(), headers: vec![], body: serde_json::to_vec(v).unwrap() } }
    pub fn raw(data: Vec<u8>, ctype: &str) -> Response { Response { status: 200, ctype: ctype.into(), headers: vec![], body: data } }
    pub fn binary(data: Vec<u8>, headers: Vec<(String, String)>, ctype: &str) -> Response { Response { status: 200, ctype: ctype.into(), headers, body: data } }
    pub fn with_status(mut self, s: u16) -> Response { self.status = s; self }
    pub fn header(mut self, k: &str, v: &str) -> Response { self.headers.push((k.into(), v.into())); self }
}
pub fn problem(status: u16, code: &str, detail: &str, instance: &str) -> Response {
    let ty = if !code.is_empty() && code != "error" { format!("{API_DOC}#error-{code}") } else { "about:blank".into() };
    let mut b = json!({"type": ty, "title": phrase(status), "status": status, "detail": detail, "code": code, "error": {"code": code, "message": detail}});
    if !instance.is_empty() { b["instance"] = json!(instance); }
    Response { status, ctype: "application/problem+json; charset=utf-8".into(), headers: vec![], body: serde_json::to_vec(&b).unwrap() }
}

pub type HResult = Result<Response, ApiError>;
pub fn ok(v: Value) -> HResult { Ok(Response::json(&v)) }

pub struct Request {
    pub method: String, pub path: String, pub target: String,
    pub params: HashMap<String, String>, pub query: HashMap<String, Vec<String>>,
    pub headers: HashMap<String, String>, pub body: Vec<u8>, pub client: String,
}
impl Request {
    pub fn json(&self) -> Result<Value, ApiError> {
        if self.body.is_empty() { return Ok(json!({})); }
        serde_json::from_slice(&self.body).map_err(|_| ApiError::coded(400, "请求体不是合法 JSON", "bad_json"))
    }
    pub fn header(&self, k: &str) -> Option<&str> { self.headers.get(&k.to_ascii_lowercase()).map(|s| s.as_str()) }
    pub fn qs(&self, k: &str) -> Option<&str> { self.query.get(k).and_then(|v| v.first()).map(|s| s.as_str()) }
    pub fn q_str(&self, k: &str, d: &str) -> String { self.qs(k).unwrap_or(d).to_string() }
    pub fn q_f64(&self, k: &str, d: f64) -> Result<f64, ApiError> {
        match self.qs(k) { None => Ok(d), Some(s) => s.parse().map_err(|_| ApiError::coded(400, format!("查询参数 {k} 无效"), "bad_param")) }
    }
    pub fn q_i64(&self, k: &str, d: i64) -> Result<i64, ApiError> {
        match self.qs(k) { None => Ok(d), Some(s) => s.parse().map_err(|_| ApiError::coded(400, format!("查询参数 {k} 无效"), "bad_param")) }
    }
    pub fn q_bool(&self, k: &str, d: bool) -> bool {
        match self.qs(k) { None => d, Some(s) => matches!(s.to_ascii_lowercase().as_str(), "1" | "true" | "yes") }
    }
}

pub enum Mount { NotHandled, Handled, HandledClose }
pub type Handler = Arc<dyn Fn(&Request) -> HResult + Send + Sync>;
pub type MountFn = Arc<dyn Fn(&Request, &mut TcpStream) -> Result<Mount, ApiError> + Send + Sync>;

struct Route { method: String, segs: Vec<String>, template: String, handler: Option<Handler>, doc: String }

pub struct Server {
    pub name: String, pub host: String, pub port: u16, pub base: String, pub title: String, pub description: String,
    routes: Vec<Route>, mounts: Vec<(String, MountFn)>, pub fallback: Option<Handler>,
}

pub fn url_decode(s: &str) -> String {
    let b = s.as_bytes();
    let mut o = Vec::with_capacity(b.len());
    let mut i = 0;
    while i < b.len() {
        match b[i] {
            b'%' if i + 2 < b.len() + 0 || i + 3 <= b.len() => {
                let h = if i + 3 <= b.len() { std::str::from_utf8(&b[i + 1..i + 3]).ok().and_then(|h| u8::from_str_radix(h, 16).ok()) } else { None };
                if let Some(v) = h { o.push(v); i += 3; } else { o.push(b'%'); i += 1; }
            }
            b'+' => { o.push(b' '); i += 1; }
            c => { o.push(c); i += 1; }
        }
    }
    String::from_utf8_lossy(&o).to_string()
}
fn path_decode(s: &str) -> String { url_decode(&s.replace('+', "%2B")) }
pub fn url_encode(s: &str) -> String {
    let mut o = String::new();
    for c in s.bytes() { if c.is_ascii_alphanumeric() || b"-_.~".contains(&c) { o.push(c as char); } else { o.push_str(&format!("%{:02X}", c)); } }
    o
}
pub fn parse_query(q: &str) -> HashMap<String, Vec<String>> {
    let mut m: HashMap<String, Vec<String>> = HashMap::new();
    for kv in q.split('&') {
        if kv.is_empty() { continue; }
        let (k, v) = kv.split_once('=').unwrap_or((kv, ""));
        m.entry(url_decode(k)).or_default().push(url_decode(v));
    }
    m
}

fn doc_parts(doc: &str) -> (String, Vec<(String, String)>, Vec<(String, String)>) {
    let re_sum = Regex::new(r"\s[{?(]|[{?]").unwrap();
    let summary = match re_sum.find(doc) { Some(m) => doc[..m.start()].trim().to_string(), None => doc.trim().to_string() };
    let summary = if summary.is_empty() { doc.trim().to_string() } else { summary };
    let mut fields = vec![];
    if let Some(i) = doc.find('{') {
        let (mut depth, mut cur, mut parts) = (0i32, String::new(), vec![]);
        for c in doc[i..].chars() {
            if c == '{' { depth += 1; if depth == 1 { continue; } }
            else if c == '}' { depth -= 1; if depth == 0 { break; } }
            if depth == 1 && c == ',' { parts.push(std::mem::take(&mut cur)); } else { cur.push(c); }
        }
        parts.push(cur);
        let re_f = Regex::new(r"(?s)^([A-Za-z_]\w*)\s*(?::\s*(.*))?$").unwrap();
        for f in parts { if let Some(m) = re_f.captures(f.trim()) { fields.push((m[1].to_string(), m.get(2).map(|x| x.as_str().trim().to_string()).unwrap_or_default())); } }
    }
    let re_q = Regex::new(r"[?&]([A-Za-z_]\w*)(?:=([^&\s)，,]*))?").unwrap();
    let query = re_q.captures_iter(doc).map(|m| (m[1].to_string(), m.get(2).map(|x| x.as_str().to_string()).unwrap_or_default())).collect();
    (summary, fields, query)
}

fn segs(t: &str) -> Vec<String> { t.trim_end_matches('/').split('/').filter(|s| !s.is_empty()).map(|s| s.to_string()).collect() }

impl Server {
    pub fn new(name: &str, port: u16, title: &str, description: &str) -> Server {
        let host = std::env::var("AGV_BIND").ok().filter(|h| !h.is_empty()).unwrap_or_else(|| "0.0.0.0".into());
        let mut s = Server { name: name.into(), host, port, base: "/api/v1".into(), title: title.into(), description: description.into(), routes: vec![], mounts: vec![], fallback: None };
        s.routes.push(Route { method: "GET".into(), segs: segs("/api/v1"), template: "/api/v1".into(), handler: None, doc: "列出全部接口 (响应头 Link 指向 OpenAPI 文档，RFC 8631)".into() });
        s.routes.push(Route { method: "GET".into(), segs: segs("/api/v1/openapi.json"), template: "/api/v1/openapi.json".into(), handler: None, doc: "本服务的 OpenAPI 3.1 文档 (由路由表生成)".into() });
        s
    }
    pub fn route(&mut self, method: &str, template: &str, doc: &str, f: impl Fn(&Request) -> HResult + Send + Sync + 'static) {
        self.routes.push(Route { method: method.to_uppercase(), segs: segs(template), template: template.into(), handler: Some(Arc::new(f)), doc: doc.into() });
    }
    pub fn mount(&mut self, prefix: &str, f: impl Fn(&Request, &mut TcpStream) -> Result<Mount, ApiError> + Send + Sync + 'static) { self.mounts.push((prefix.into(), Arc::new(f))); }

    fn index(&self) -> Response {
        let routes: Vec<Value> = self.routes.iter().map(|r| json!({"method": r.method, "path": r.template, "doc": r.doc})).collect();
        let b = json!({"service": self.name, "api_version": API_VERSION, "openapi": format!("{}/openapi.json", self.base), "routes": routes});
        Response::json(&b).header("Link", &format!("<{}/openapi.json>; rel=\"service-desc\"; type=\"application/openapi+json\", <{API_DOC}>; rel=\"service-doc\"", self.base))
    }

    pub fn openapi(&self) -> Value {
        let (mut paths, mut tags): (Map<String, Value>, Vec<String>) = (Map::new(), vec![]);
        let re_name = Regex::new(r"\{(\w+)\}").unwrap();
        let re_non = Regex::new(r"[^A-Za-z0-9]+").unwrap();
        for r in &self.routes {
            let (summary, fields, query) = doc_parts(&r.doc);
            let rest = r.template.strip_prefix(&self.base).unwrap_or(&r.template);
            let tag = rest.split('/').filter(|x| !x.is_empty() && !x.starts_with('{') && !["api", "hub", "v1", "v2"].contains(x)).next().unwrap_or("meta").to_string();
            if !tags.contains(&tag) { tags.push(tag.clone()); }
            let opid: String = format!("{}_{}", r.method.to_lowercase(), re_non.replace_all(&r.template, "_").trim_matches('_')).chars().take(120).collect();
            let mut op = json!({"operationId": opid, "summary": summary, "tags": [tag], "responses": {
                "200": {"description": "成功 (JSON；少数接口返回二进制/文本，见说明)", "content": {"application/json": {"schema": {}}}},
                "default": {"description": "错误 (RFC 9457)", "content": {"application/problem+json": {"schema": {"$ref": "#/components/schemas/Problem"}}}}}});
            if !r.doc.is_empty() && r.doc != summary { op["description"] = json!(r.doc); }
            let path_names: Vec<String> = re_name.captures_iter(&r.template).map(|m| m[1].to_string()).collect();
            let mut params: Vec<Value> = path_names.iter().map(|n| json!({"name": n, "in": "path", "required": true, "schema": {"type": "string"}})).collect();
            for (n, v) in &query {
                if path_names.contains(n) || params.iter().any(|p| p["name"] == json!(n) && p["in"] == json!("query")) { continue; }
                let mut p = json!({"name": n, "in": "query", "required": false, "schema": {"type": "string"}});
                if !v.is_empty() { p["description"] = json!(format!("例如 {v}")); }
                params.push(p);
            }
            if !params.is_empty() { op["parameters"] = json!(params); }
            if ["POST", "PUT", "PATCH"].contains(&r.method.as_str()) {
                if r.template.ends_with("/upload") || r.doc.contains("请求体为文件") || r.doc.contains("请求体为 .tgz") {
                    op["requestBody"] = json!({"required": true, "description": "文件内容", "content": {"application/octet-stream": {"schema": {"type": "string", "contentMediaType": "application/octet-stream"}}}});
                } else {
                    let props: Map<String, Value> = fields.iter().map(|(n, d)| (n.clone(), if d.is_empty() { json!({}) } else { json!({"description": d}) })).collect();
                    let sch = if r.doc.contains("请求体为数组") {
                        let mut s = json!({"type": "array", "items": {"type": "object"}});
                        if !fields.is_empty() { s["items"]["properties"] = Value::Object(props); }
                        s
                    } else { let mut s = json!({"type": "object"}); if !fields.is_empty() { s["properties"] = Value::Object(props); } s };
                    op["requestBody"] = json!({"required": !fields.is_empty(), "content": {"application/json": {"schema": sch}}});
                }
            }
            let e = paths.entry(r.template.clone()).or_insert_with(|| json!({}));
            e[r.method.to_lowercase()] = op;
        }
        let mut spec = json!({"openapi": "3.1.0",
            "info": {"title": self.title, "version": API_VERSION, "description": if self.description.is_empty() { format!("{} 服务接口", self.name) } else { self.description.clone() }},
            "servers": [{"url": "/", "description": "相对当前服务地址"}],
            "tags": tags.iter().map(|t| json!({"name": t})).collect::<Vec<_>>(), "paths": paths,
            "components": {"schemas": {"Problem": {"type": "object", "description": "RFC 9457 Problem Details；code / error 为本项目扩展",
                "properties": {"type": {"type": "string", "format": "uri-reference"}, "title": {"type": "string"}, "status": {"type": "integer"}, "detail": {"type": "string"},
                    "instance": {"type": "string", "format": "uri-reference"}, "code": {"type": "string", "description": "机器可读错误码"},
                    "error": {"type": "object", "description": "兼容旧客户端: {code, message}"}}}}}});
        if !self.mounts.is_empty() { spec["x-raw-mounts"] = json!(self.mounts.iter().map(|(p, _)| p.clone()).collect::<Vec<_>>()); }
        spec
    }

    pub fn dispatch(&self, req: &mut Request, method: &str) -> Result<Response, ApiError> {
        let path_segs: Vec<&str> = req.path.split('/').filter(|s| !s.is_empty()).collect();
        let mut allowed: Vec<&str> = vec![];
        for r in &self.routes {
            if r.segs.len() != path_segs.len() { continue; }
            let mut params = HashMap::new();
            let mut ok = true;
            for (t, p) in r.segs.iter().zip(&path_segs) {
                if t.starts_with('{') && t.ends_with('}') { params.insert(t[1..t.len() - 1].to_string(), path_decode(p)); }
                else if t != p { ok = false; break; }
            }
            if !ok { continue; }
            if r.method != method { allowed.push(&r.method); continue; }
            req.params = params;
            return match &r.handler {
                Some(h) => h(req),
                None => if r.template.ends_with("openapi.json") { Ok(Response::json(&self.openapi())) } else { Ok(self.index()) },
            };
        }
        if !allowed.is_empty() {
            allowed.sort(); allowed.dedup();
            return Err(ApiError::coded(405, format!("{method} 不支持，可用: {}", allowed.join(",")), "method_not_allowed"));
        }
        if method == "GET" { if let Some(f) = &self.fallback { return f(req); } }
        Err(ApiError::coded(404, format!("未找到资源 {}", req.path), "not_found"))
    }

    fn write_resp(&self, s: &mut TcpStream, r: &Response, head: bool, keep: bool) -> std::io::Result<()> {
        let mut h = format!("HTTP/1.1 {} {}\r\nContent-Type: {}\r\nContent-Length: {}\r\nAccess-Control-Allow-Origin: *\r\nAccess-Control-Expose-Headers: *\r\n", r.status, phrase(r.status), r.ctype, r.body.len());
        if !r.headers.iter().any(|(k, _)| k.eq_ignore_ascii_case("cache-control")) { h.push_str("Cache-Control: no-cache, no-store\r\n"); }
        for (k, v) in &r.headers { h.push_str(&format!("{k}: {v}\r\n")); }
        if !keep { h.push_str("Connection: close\r\n"); }
        h.push_str("\r\n");
        let mut out = h.into_bytes();
        if !head { out.extend_from_slice(&r.body); }
        s.write_all(&out)
    }

    fn handle_conn(self: &Arc<Self>, mut s: TcpStream) {
        let _ = s.set_nodelay(true);
        let peer = s.peer_addr().map(|a| a.to_string()).unwrap_or_default();
        let Ok(rs) = s.try_clone() else { return };
        let mut rd = BufReader::new(rs);
        loop {
            let mut line = String::new();
            match rd.read_line(&mut line) { Ok(0) | Err(_) => return, Ok(_) => {} }
            let line = line.trim_end();
            if line.is_empty() { continue; }
            let mut it = line.split_whitespace();
            let (Some(method), Some(target)) = (it.next(), it.next()) else { return };
            let version = it.next().unwrap_or("HTTP/1.0");
            let mut headers: HashMap<String, String> = HashMap::new();
            loop {
                let mut h = String::new();
                match rd.read_line(&mut h) { Ok(0) | Err(_) => return, Ok(_) => {} }
                let h = h.trim_end();
                if h.is_empty() { break; }
                if let Some((k, v)) = h.split_once(':') { headers.insert(k.trim().to_ascii_lowercase(), v.trim().to_string()); }
            }
            let n: usize = headers.get("content-length").and_then(|v| v.parse().ok()).unwrap_or(0);
            let mut body = vec![0u8; n];
            if n > 0 && rd.read_exact(&mut body).is_err() { return; }
            let conn_h = headers.get("connection").map(|s| s.to_ascii_lowercase()).unwrap_or_default();
            let keep = if version == "HTTP/1.1" { conn_h != "close" } else { conn_h == "keep-alive" };
            let (path, qs) = target.split_once('?').unwrap_or((target, ""));
            let path_n = if path.len() > 1 { path.trim_end_matches('/') } else { path };
            let mut req = Request { method: method.into(), path: path_n.into(), target: target.into(), params: HashMap::new(), query: parse_query(qs), headers, body, client: peer.clone() };
            let inst = path.to_string();
            if method == "OPTIONS" {
                let r = "HTTP/1.1 204 No Content\r\nAccess-Control-Allow-Origin: *\r\nAccess-Control-Allow-Methods: GET,POST,PUT,PATCH,DELETE,OPTIONS\r\nAccess-Control-Allow-Headers: Content-Type, X-Lock-Token, X-Node-Key, X-User, X-Admin-Token\r\nContent-Length: 0\r\n\r\n";
                if s.write_all(r.as_bytes()).is_err() { return; }
                if !keep { return; }
                continue;
            }
            let mut handled: Option<Mount> = None;
            for (prefix, f) in &self.mounts {
                if target.starts_with(prefix.as_str()) {
                    match f(&req, &mut s) {
                        Ok(Mount::NotHandled) => {}
                        Ok(m) => { handled = Some(m); break; }
                        Err(e) => { let r = problem(e.status, &e.code, &e.message, &inst); let _ = self.write_resp(&mut s, &r, false, keep); handled = Some(Mount::Handled); break; }
                    }
                }
            }
            match handled {
                Some(Mount::HandledClose) => return,
                Some(_) => { if !keep { return; } continue; }
                None => {}
            }
            let head = method == "HEAD";
            let m = if head { "GET" } else { method };
            let res = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| self.dispatch(&mut req, m)));
            let resp = match res {
                Ok(Ok(r)) => r,
                Ok(Err(e)) => problem(e.status, &e.code, &e.message, &inst),
                Err(_) => problem(500, "internal", "internal error", &inst),
            };
            if self.write_resp(&mut s, &resp, head, keep).is_err() { return; }
            if !keep { return; }
        }
    }

    pub fn serve_forever(self: Arc<Self>) -> std::io::Result<()> {
        let l = TcpListener::bind((self.host.as_str(), self.port))?;
        for c in l.incoming() {
            let Ok(c) = c else { continue };
            let me = self.clone();
            std::thread::Builder::new().name("http".into()).stack_size(512 * 1024).spawn(move || me.handle_conn(c)).ok();
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------- client
pub struct Client { pub host: String, pub port: u16, pub prefix: String, pub timeout: f64 }
pub struct ClientResp { pub status: u16, pub headers: HashMap<String, String>, pub body: Vec<u8> }

thread_local! { static CONNS: std::cell::RefCell<HashMap<String, BufReader<TcpStream>>> = std::cell::RefCell::new(HashMap::new()); }

impl Client {
    pub fn new(base_url: &str, timeout: f64) -> Client {
        let u = base_url.trim_start_matches("http://");
        let (hp, pre) = u.split_once('/').map(|(a, b)| (a, format!("/{}", b.trim_end_matches('/')))).unwrap_or((u, String::new()));
        let (h, p) = hp.rsplit_once(':').map(|(h, p)| (h.to_string(), p.parse().unwrap_or(80))).unwrap_or((hp.to_string(), 80));
        Client { host: h, port: p, prefix: pre, timeout }
    }
    fn key(&self) -> String { format!("{}:{}", self.host, self.port) }
    pub fn request(&self, method: &str, path: &str, body: Option<&Value>, timeout: Option<f64>, accept: &str) -> Result<ClientResp, String> {
        let t = timeout.unwrap_or(self.timeout);
        let data = body.map(|b| serde_json::to_vec(b).unwrap());
        let mut last = String::new();
        for _ in 0..2 {
            let r = CONNS.with(|c| {
                let mut c = c.borrow_mut();
                let key = self.key();
                if !c.contains_key(&key) {
                    let addr = format!("{}:{}", self.host, self.port);
                    let s = TcpStream::connect(&addr).map_err(|e| e.to_string())?;
                    let _ = s.set_nodelay(true);
                    c.insert(key.clone(), BufReader::new(s));
                }
                let rd = c.get_mut(&key).unwrap();
                let d = std::time::Duration::from_secs_f64(t.max(0.01));
                let _ = rd.get_ref().set_read_timeout(Some(d));
                let _ = rd.get_ref().set_write_timeout(Some(d));
                let mut req = format!("{method} {}{path} HTTP/1.1\r\nHost: {}:{}\r\nAccept: {accept}\r\nConnection: keep-alive\r\n", self.prefix, self.host, self.port);
                if let Some(b) = &data { req.push_str(&format!("Content-Type: application/json\r\nContent-Length: {}\r\n", b.len())); }
                else if method != "GET" { req.push_str("Content-Length: 0\r\n"); }
                req.push_str("\r\n");
                let mut out = req.into_bytes();
                if let Some(b) = &data { out.extend_from_slice(b); }
                rd.get_mut().write_all(&out).map_err(|e| e.to_string())?;
                let mut line = String::new();
                if rd.read_line(&mut line).map_err(|e| e.to_string())? == 0 { return Err("closed".to_string()); }
                let status: u16 = line.split_whitespace().nth(1).and_then(|s| s.parse().ok()).ok_or("bad status")?;
                let mut headers = HashMap::new();
                loop {
                    let mut h = String::new();
                    if rd.read_line(&mut h).map_err(|e| e.to_string())? == 0 { return Err("closed".to_string()); }
                    let h = h.trim_end();
                    if h.is_empty() { break; }
                    if let Some((k, v)) = h.split_once(':') { headers.insert(k.trim().to_ascii_lowercase(), v.trim().to_string()); }
                }
                let n: usize = headers.get("content-length").and_then(|v| v.parse().ok()).unwrap_or(0);
                let mut body = vec![0u8; n];
                rd.read_exact(&mut body).map_err(|e| e.to_string())?;
                if headers.get("connection").map(|v| v.eq_ignore_ascii_case("close")).unwrap_or(false) { c.remove(&key); }
                Ok(ClientResp { status, headers, body })
            });
            match r {
                Ok(r) => return Ok(r),
                Err(e) => { CONNS.with(|c| c.borrow_mut().remove(&self.key())); last = e; }
            }
        }
        Err(format!("{method} {path} 失败: {last}"))
    }
    pub fn json(&self, method: &str, path: &str, body: Option<&Value>, timeout: Option<f64>) -> Result<Value, ApiError> {
        let r = self.request(method, path, body, timeout, "application/json").map_err(|e| ApiError::coded(503, e, "unreachable"))?;
        let v: Value = if r.body.is_empty() { json!({}) } else { serde_json::from_slice(&r.body).unwrap_or(Value::Null) };
        if r.status >= 400 {
            let msg = v.pointer("/error/message").and_then(|m| m.as_str()).unwrap_or("");
            return Err(ApiError::coded(r.status, format!("{method} {path}: {msg}"), "remote"));
        }
        Ok(v)
    }
    pub fn get(&self, path: &str) -> Result<Value, ApiError> { self.json("GET", path, None, None) }
    pub fn put(&self, path: &str, b: &Value) -> Result<Value, ApiError> { self.json("PUT", path, Some(b), None) }
    pub fn post(&self, path: &str, b: &Value) -> Result<Value, ApiError> { self.json("POST", path, Some(b), None) }
}
