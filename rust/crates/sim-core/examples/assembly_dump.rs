//! cargo run --example assembly_dump -- spec.json   → prints the robot assembly JSON (archetype, links, missing report)
fn main() {
    let p = std::env::args().nth(1).expect("spec.json");
    let spec: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(p).unwrap()).unwrap();
    println!("{}", serde_json::to_string_pretty(&sim_core::assembly::build(&spec)).unwrap());
}
