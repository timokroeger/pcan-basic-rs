use std::env;

fn main() {
    println!("cargo:rustc-link-lib=PCANBasic");

    let manifest_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
    let target = env::var("TARGET").unwrap();
    println!(
        "cargo:rustc-link-search={}/PCANBasic/{}",
        manifest_dir, target
    );
}
