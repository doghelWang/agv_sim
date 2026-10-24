fn main() {
    let a: Vec<String> = std::env::args().collect();
    let load = a.get(2).map(|s| s.as_str()).unwrap_or("full");
    let d = cmodel::decode_cmodel_bytes(&std::fs::read(&a[1]).unwrap()).unwrap();
    let name = std::path::Path::new(&a[1]).file_name().unwrap().to_string_lossy().to_string();
    println!("{}", serde_json::to_string(&cmodel::spec::extract_robot_spec(&d["CompDesc"], &name, load, None)).unwrap());
}
