use std::env;
use std::fs::File;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::Command;

fn main() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");

    // Watch root aircraft configuration
    println!("cargo:rerun-if-changed=../../wingfc.yaml");

    let yaml_path = Path::new("../../wingfc.yaml");
    if yaml_path.exists() {
        let gen_config = out.join("generated_config.rs");
        let status = Command::new("python3")
            .arg("../../scripts/builder.py")
            .arg("generate")
            .arg("--out-config")
            .arg(&gen_config)
            .status();
        match status {
            Ok(s) if s.success() => {},
            Ok(s) => panic!("wingfc.yaml validation or config generation failed with exit code: {:?}", s.code()),
            Err(e) => eprintln!("cargo:warning=Failed to execute python3 ../../scripts/builder.py generate: {}", e),
        }
    }
}
