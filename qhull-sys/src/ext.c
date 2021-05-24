#include "libqhull_r/geom_r.h"
#include "libqhull_r/io_r.h"
#include "libqhull_r/libqhull_r.h"
#include "libqhull_r/mem_r.h"
#include "libqhull_r/merge_r.h"
#include "libqhull_r/poly_r.h"
#include "libqhull_r/qhull_ra.h"
#include "libqhull_r/qset_r.h"
#include "libqhull_r/random_r.h"
#include "libqhull_r/stat_r.h"
#include "libqhull_r/user_r.h"


/* We don't want to depend on bindgen/libclang, so implement a few helpers to
   ensure we don't need to know the layout fo qhT.
   */
qhT *rqh_malloc() {
    return malloc(sizeof(qhT));
}

unsigned int* rqh_get_NOerrexit(qhT *qh) {
    return &qh->NOerrexit;
}

facetT** rqh_get_facet_list(qhT *qh) {
    return &qh->facet_list;
}

vertexT** rqh_get_vertex_list(qhT *qh) {
    return &qh->vertex_list;
}

boolT* rqh_get_PROJECTdelaunay(qhT *qh) {
    return &qh->PROJECTdelaunay;
}


FILE* rqh_stdin() {
    return stdin;
}

FILE* rqh_stdout() {
    return stdout;
}

FILE* rqh_stderr() {
    return stderr;
}
