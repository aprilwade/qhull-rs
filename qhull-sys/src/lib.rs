
// #[cfg(all(target_arch = "aarch64", target_os = "macos"))]
// #[path = "aarch64-apple-darwin.rs"]
// mod bindings;

// #[cfg(all(target_arch = "x86_64", target_os = "macos"))]
// #[path = "x86_64-apple-darwin.rs"]
// mod bindings;

mod binding {
    #![allow(non_camel_case_types)]
    #![allow(non_snake_case)]
    #![allow(non_upper_case_globals)]

    #[repr(C)]
    pub struct qhT {
        _empty: [u8; 0],
    }
    use libc::FILE;
    include!("binding.rs");
}

pub use binding:: *;
