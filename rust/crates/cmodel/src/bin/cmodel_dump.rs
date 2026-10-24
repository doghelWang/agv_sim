fn main() {
    let p = std::env::args().nth(1).expect("usage: cmodel_dump file.cmodel");
    let d = cmodel::decode_cmodel_bytes(&std::fs::read(p).unwrap()).unwrap();
    println!("{}", serde_json::to_string(&serde_json::Value::Object(d)).unwrap());
}
