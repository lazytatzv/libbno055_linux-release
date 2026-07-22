fn main() {
    cc::Build::new()
        .cpp(true)
        .std("c++17")
        .include("cpp/include")
        .file("cpp/src/bno055.cpp")
        .file("cpp/src/bno055_c.cpp")
        .compile("bno055-linux");

    println!("cargo:rerun-if-changed=cpp/include/libbno055-linux/bno055_c.h");
    println!("cargo:rerun-if-changed=cpp/include/libbno055-linux/bno055.hpp");
    println!("cargo:rerun-if-changed=cpp/src/bno055.cpp");
    println!("cargo:rerun-if-changed=cpp/src/bno055_c.cpp");
}
