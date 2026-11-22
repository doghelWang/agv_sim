fn main() {
    let p = std::env::args().nth(1).unwrap();
    let d = cmodel::decode_cmodel_bytes(&std::fs::read(&p).unwrap()).unwrap();
    let spec = cmodel::spec::extract_robot_spec(&d["CompDesc"], "x", "full", None);
    println!("{}", serde_json::to_string(&spec).unwrap());
}
