const C_FILES: &[&str] = &[
    "qhull/src/libqhull_r/global_r.c",
    "qhull/src/libqhull_r/stat_r.c",
    "qhull/src/libqhull_r/geom2_r.c",
    "qhull/src/libqhull_r/poly2_r.c",
    "qhull/src/libqhull_r/merge_r.c",
    "qhull/src/libqhull_r/libqhull_r.c",
    "qhull/src/libqhull_r/geom_r.c",
    "qhull/src/libqhull_r/poly_r.c",
    "qhull/src/libqhull_r/qset_r.c",
    "qhull/src/libqhull_r/mem_r.c",
    "qhull/src/libqhull_r/random_r.c",
    "qhull/src/libqhull_r/usermem_r.c",
    "qhull/src/libqhull_r/userprintf_r.c",
    "qhull/src/libqhull_r/io_r.c",
    "qhull/src/libqhull_r/user_r.c",
    "qhull/src/libqhull_r/rboxlib_r.c",
    "qhull/src/libqhull_r/userprintf_rbox_r.c",
    "src/ext.c",
];

fn main() {
    let mut build = cc::Build::new();
    build.include("qhull/src/")
        // .flag("-O3")
        .pic(true)
        .flag("-ansi");
    for c_file in C_FILES {
        build.file(c_file);
    }
    build.compile("qhull_r");
}
