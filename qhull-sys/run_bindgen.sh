bindgen \
    --allowlist-function qh_setsize \
    --allowlist-function qh_setappend \
    --allowlist-function qh_init_A \
    --allowlist-function qh_init_B \
    --allowlist-function qh_checkflags \
    --allowlist-function qh_initflags \
    --allowlist-function qh_option \
    --allowlist-function qh_freeqhull \
    --allowlist-function qh_memfreeshort \
    --allowlist-function qh_qhull \
    --allowlist-function qh_check_output \
    --allowlist-function qh_produce_output \
    --allowlist-function qh_triangulate \
    --allowlist-function qh_checkpolygon \
    --allowlist-function qh_findgood_all \
    --allowlist-function qh_appendprint \
    --allowlist-function qh_pointvertex \
    --allowlist-function qh_readpoints \
    --allowlist-function qh_zero \
    --allowlist-function qh_pointid \
    --allowlist-function qh_nearvertex \
    --allowlist-function qh_addpoint \
    --allowlist-function qh_findbestfacet \
    --allowlist-function qh_setdelaunay \
    --allowlist-function qh_sethalfspace_all \
    --allowlist-function qh_eachvoronoi_all \
    --allowlist-function qh_order_vertexneighbors \
    --allowlist-function qh_compare_facetvisit \
    --allowlist-function qh_facetcenter \
    --allowlist-function qh_getarea \
    --allowlist-function qh_check_maxout \
    --allowlist-function qh_memfre \
    --allowlist-function qh_distplane \
    --allowlist-function qhull_misc_lib_chec \
    --allowlist-function 'rqh_.*' \
    --blocklist-type jmp_buf \
    --blocklist-type qhT \
    --blocklist-type FILE \
    --no-doc-comments \
    --size_t-is-usize \
    --no-layout-tests \
    src/ext.c \
    -- -I qhull/src/

    # --allowlist-function setjmp \
    # --allowlist-function longjmp \
