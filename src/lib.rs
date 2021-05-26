use qhull_sys::qhT;

use std::fmt;
use std::iter::{self, FromIterator};
use std::marker::PhantomData;
use std::ptr::{self, NonNull};

use nalgebra::{OMatrix, OVector};
use nalgebra::base::allocator::{Allocator, Reallocator};
use nalgebra::{Const, DefaultAllocator, DimAdd, DimName, DimSum, U1};


#[derive(Debug)]
pub struct ConvexHull<N: DimName> {
    qh: NonNull<qhT>,
    pd: PhantomData<N>,
}

impl<const N: usize> ConvexHull<Const<N>> {
    pub fn from_arrays(slice: &[[f64; N]]) -> Self {
        unsafe {
            let num_points = slice.len();
            let len = slice.len() * std::mem::size_of::<[f64; N]>();
            let buf = libc::malloc(len);
            libc::memcpy(buf, slice.as_ptr().cast(), len);
            Self::init_shared(buf.cast(), num_points)
        }
    }
}


impl<N: DimName> ConvexHull<N> {
    pub fn from_vectors(slice: &[OVector<f64, N>]) -> Self
        where DefaultAllocator: Allocator<f64, N>,
    {
        unsafe {
            let num_points = slice.len();
            let len = slice.len() * std::mem::size_of::<OVector<f64, N>>();
            let buf = libc::malloc(len);
            libc::memcpy(buf, slice.as_ptr().cast(), len);
            Self::init_shared(buf.cast(), num_points)
        }
    }

    unsafe fn init_shared(buf: *mut f64, len: usize) -> Self {
        let qh = qhull_sys::rqh_malloc();
        // TODO:
        let stdin = qhull_sys::rqh_stdin();
        let stdout = qhull_sys::rqh_stdout();
        let stderr = qhull_sys::rqh_stderr();
        qhull_sys::qh_init_A(qh, stdin, stdout, stderr, 0, ptr::null_mut());
        *qhull_sys::rqh_get_NOerrexit(qh) = 0;
        // let mut cmd = *b"qhull Qbb Qc Qz Qx Q12\0";
        // let mut cmd = *b"qhull Qbb Qc Qx Q12\0";
        let mut cmd = *b"qhull Qc Qx Q12\0";
        qhull_sys::qh_initflags(qh, cmd.as_mut_ptr().cast());
        qhull_sys::qh_init_B(qh, buf, len as _, N::dim() as _, 1);
        qhull_sys::qh_qhull(qh);
        qhull_sys::qh_triangulate(qh);
        ConvexHull {
            qh: NonNull::new(qh).unwrap(),
            pd: PhantomData,
        }
    }

    pub fn facets<'a>(&'a self) -> FacetListIterator<'a, N> {
        FacetListIterator {
            ptr: NonNull::new(unsafe { qhull_sys::rqh_get_facet_list(self.qh.as_ptr()).read() }),
            pd: PhantomData,
        }
    }

    pub fn vertices<'a>(&'a self) -> VertexListIterator<'a, N> {
        VertexListIterator {
            ptr: NonNull::new(unsafe { qhull_sys::rqh_get_vertex_list(self.qh.as_ptr()).read() }),
            pd: PhantomData,
        }
    }
}

impl<N: DimName> FromIterator<OVector<f64, N>> for ConvexHull<N>
    where DefaultAllocator: Allocator<f64, N>,
{
    fn from_iter<T: IntoIterator<Item = OVector<f64, N>>>(iter: T) -> Self {
        // TODO: Be smarter about how this is implemented. Ideally we wouldn't need Vec at all
        //       Or perhaps we could use a custom allocator to ensure we can safely hand ownership
        //       over to the qhull library
        let v: Vec<_> = iter.into_iter().collect();
        unsafe {
            let num_points = v.len();
            let len = v.len() * std::mem::size_of::<OVector<f64, N>>();
            let buf = libc::malloc(len);
            libc::memcpy(buf, v.as_ptr().cast(), len);

            // Drop `v` to free it's memory
            let _ = v;
            Self::init_shared(buf.cast(), num_points)
        }
    }
}

impl<N: DimName> Drop for ConvexHull<N> {
    fn drop(&mut self) {
        unsafe {
            qhull_sys::qh_freeqhull(self.qh.as_ptr(), 0);

            let mut tolong = 0;
            let mut currlong = 0;
            qhull_sys::qh_memfreeshort(self.qh.as_ptr(), &mut currlong, &mut tolong);

            libc::free(self.qh.as_ptr().cast());
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Vertex<'a, N: DimName> {
    ptr: NonNull<qhull_sys::vertexT>,
    pd: PhantomData<(&'a (), N)>,
}

impl<'a, N: DimName> Vertex<'a, N> {
    pub fn point(&self) -> &'a [f64] {
        unsafe {
            let ptr = ptr::addr_of!((*self.ptr.as_ptr()).point).read();
            std::slice::from_raw_parts(ptr, N::dim())
        }
    }

    pub fn neighbors(&self) -> SetIter<'a, Facet<'a, N>> {
        let ptr = unsafe {
            let set_ptr = ptr::addr_of!((*self.ptr.as_ptr()).neighbors).read();
            if set_ptr.is_null() {
                panic!("facetT::neighbors pointer was NULL!");
            }
            NonNull::new_unchecked(ptr::addr_of_mut!((*set_ptr).e).cast())
        };
        SetIter {
            ptr,
            pd: PhantomData,
        }
    }

    pub fn id(&self) -> u64 {
        unsafe {
            ptr::addr_of!((*self.ptr.as_ptr()).id).read().into()
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Facet<'a, N: DimName> {
    ptr: NonNull<qhull_sys::facetT>,
    pd: PhantomData<(&'a (), N)>,
}

impl<'a, N: DimName> Facet<'a, N> {
    pub fn vertices(&self) -> SetIter<'a, Vertex<'a, N>> {
        let ptr = unsafe {
            let set_ptr = ptr::addr_of!((*self.ptr.as_ptr()).vertices).read();
            if set_ptr.is_null() {
                panic!("facetT::vertices pointer was NULL!");
            }
            NonNull::new_unchecked(ptr::addr_of_mut!((*set_ptr).e).cast())
        };
        SetIter {
            ptr,
            pd: PhantomData,
        }
    }

    /// ... the kth neighbor is opposite the kth vertex ...
    pub fn neighbors(&self) -> SetIter<'a, Facet<'a, N>> {
        let ptr = unsafe {
            let set_ptr = ptr::addr_of!((*self.ptr.as_ptr()).neighbors).read();
            if set_ptr.is_null() {
                panic!("facetT::neighbors pointer was NULL!");
            }
            NonNull::new_unchecked(ptr::addr_of_mut!((*set_ptr).e).cast())
        };
        SetIter {
            ptr,
            pd: PhantomData,
        }
    }

    pub fn id(&self) -> u64 {
        unsafe {
            ptr::addr_of!((*self.ptr.as_ptr()).id).read().into()
        }
    }

    /// Returns true if the facet is a . Always false for the facets of a convex hull.
    pub fn is_upper_delaunay(&self) -> bool {
        unsafe {
            (*self.ptr.as_ptr()).upperdelaunay() == 1
        }
    }

    pub fn plane_distance(&self, point: &OVector<f64, N>) -> f64
        where DefaultAllocator: Allocator<f64, N>,
    {
        let mut sum = unsafe { ptr::addr_of!((*self.ptr.as_ptr()).offset).read() };
        let normal_ptr = unsafe { ptr::addr_of!((*self.ptr.as_ptr()).normal).read() };
        for (i, f) in point.iter().enumerate() {
            sum += unsafe { normal_ptr.offset(i as isize).read() } * f;
        }
        sum
    }
}

unsafe fn next_set_element(idk: &mut NonNull<*mut qhull_sys::setelemT>)
    -> Option<NonNull<qhull_sys::setelemT>>
{
    let ptr = NonNull::new(idk.as_ptr().read().cast());
    if let Some(ptr) = ptr {
        *idk = NonNull::new_unchecked(idk.as_ptr().add(1));
        Some(ptr)
    } else {
        None
    }
}

#[doc(hidden)]
pub trait FromSetElem {
    unsafe fn from_setelem(ptr: NonNull<qhull_sys::setelemT>) -> Self;
}

impl<'a, N: DimName> FromSetElem for Vertex<'a, N> {
    unsafe fn from_setelem(ptr: NonNull<qhull_sys::setelemT>) -> Self {
        Vertex {
            ptr: ptr.cast(),
            pd: PhantomData,
        }
    }
}

impl<'a, N: DimName> FromSetElem for Facet<'a, N> {
    unsafe fn from_setelem(ptr: NonNull<qhull_sys::setelemT>) -> Self {
        Facet {
            ptr: ptr.cast(),
            pd: PhantomData,
        }
    }
}


pub struct SetIter<'a, T> {
    ptr: NonNull<*mut qhull_sys::setelemT>,
    pd: PhantomData<(&'a (), *const T)>,
}

impl<'a, T> Clone for SetIter<'a, T> {
    fn clone(&self) -> Self {
        SetIter {
            ptr: self.ptr.clone(),
            pd: PhantomData,
        }
    }
}

impl<'a, T> Copy for SetIter<'a, T> { }

impl<'a, T> fmt::Debug for SetIter<'a, T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SetIter")
            .field("ptr", &self.ptr)
            .finish()
    }
}

impl<'a, T> Iterator for SetIter<'a, T>
    where T: FromSetElem,
{
    type Item = T;

    fn next(&mut self) -> Option<Self::Item> {
        unsafe {
            next_set_element(&mut self.ptr)
                .map(|ptr| T::from_setelem(ptr))
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub struct FacetListIterator<'a, N: DimName> {
    ptr: Option<NonNull<qhull_sys::facetT>>,
    pd: PhantomData<(&'a (), N)>,
}

impl<'a, N: DimName> Iterator for FacetListIterator<'a, N> {
    type Item = Facet<'a, N>;

    fn next(&mut self) -> Option<Self::Item> {
        if let Some(ptr) = self.ptr {
            unsafe {
                // AHHHHH! Why?
                if ptr::addr_of!((*ptr.as_ptr()).id).read() == 0 {
                    return None
                }
                self.ptr = NonNull::new(ptr::addr_of!((*ptr.as_ptr()).next).read());
            }
            Some(Facet {
                ptr: ptr,
                pd: PhantomData,
            })
        } else {
            None
        }
    }
}


#[derive(Debug, Copy, Clone)]
pub struct VertexListIterator<'a, N: DimName> {
    ptr: Option<NonNull<qhull_sys::vertexT>>,
    pd: PhantomData<(&'a (), N)>,
}

impl<'a, N: DimName> Iterator for VertexListIterator<'a, N> {
    type Item = Vertex<'a, N>;

    fn next(&mut self) -> Option<Self::Item> {
        if let Some(ptr) = self.ptr {
            unsafe {
                // AHHHHH! Why?
                if ptr::addr_of!((*ptr.as_ptr()).id).read() == 0 {
                    return None
                }
                self.ptr = NonNull::new(ptr::addr_of!((*ptr.as_ptr()).next).read());
            }
            Some(Vertex {
                ptr: ptr,
                pd: PhantomData,
            })
        } else {
            None
        }
    }
}

#[derive(Debug)]
pub struct Delaunay<N: DimName>
    where DefaultAllocator: Allocator<f64, N>,
          N: DimAdd<U1>,
          DimSum<N, U1>: DimName,
{
    cv: ConvexHull<DimSum<N, U1>>,
    max_bounds: OVector<f64, N>,
    min_bounds: OVector<f64, N>,
}

impl<const N: usize> Delaunay<Const<N>>
    where DefaultAllocator: Allocator<f64, Const<N>>,
          Const<N>: DimAdd<U1>,
          DimSum<Const<N>, U1>: DimName,
{
    pub fn from_arrays(slice: &[[f64; N]]) -> Self {
        unsafe {
            let num_points = slice.len();
            let len = slice.len() * std::mem::size_of::<[f64; N]>();
            let buf = libc::malloc(len);
            libc::memcpy(buf, slice.as_ptr().cast(), len);
            Self::init_shared(buf.cast(), num_points)
        }
    }
}

impl<N: DimName> Delaunay<N>
    where DefaultAllocator: Allocator<f64, N>,
          N: DimAdd<U1>,
          DimSum<N, U1>: DimName,
{
    pub fn from_vectors(slice: &[OVector<f64, N>]) -> Self {
        unsafe {
            let num_points = slice.len();
            let len = slice.len() * std::mem::size_of::<OVector<f64, N>>();
            let buf = libc::malloc(len);
            libc::memcpy(buf, slice.as_ptr().cast(), len);
            Self::init_shared(buf.cast(), num_points)
        }
    }

    unsafe fn init_shared(buf: *mut f64, len: usize) -> Self {
        let qh = qhull_sys::rqh_malloc();
        // TODO:
        let stdin = qhull_sys::rqh_stdin();
        let stdout = qhull_sys::rqh_stdout();
        let stderr = qhull_sys::rqh_stderr();
        qhull_sys::qh_init_A(qh, stdin, stdout, stderr, 0, ptr::null_mut());
        *qhull_sys::rqh_get_NOerrexit(qh) = 0;
        // let mut cmd = *b"qhull Qbb Qc Qz Qx Q12\0";
        // let mut cmd = *b"qhull Qbb Qc Qx Q12\0";
        let mut cmd = *b"qhull d Tv Qbb Qc Qz Qx Q12\0";
        qhull_sys::qh_initflags(qh, cmd.as_mut_ptr().cast());
        *qhull_sys::rqh_get_PROJECTdelaunay(qh) = 1;
        qhull_sys::qh_init_B(qh, buf, len as _, N::dim() as _, 1);
        qhull_sys::qh_qhull(qh);
        qhull_sys::qh_triangulate(qh);

        let cv = ConvexHull {
            qh: NonNull::new(qh).unwrap(),
            pd: PhantomData,
        };
        let mut max_bounds = OVector::from_element(f64::MIN);
        let mut min_bounds = OVector::from_element(f64::MAX);
        for vertex in cv.vertices() {
            for ((n, max), min) in vertex.point().iter().zip(&mut max_bounds).zip(&mut min_bounds) {
                if n > max {
                    *max = *n
                }
                if n < min {
                    *min = *n
                }
            }
        }

        Delaunay {
            cv,
            max_bounds,
            min_bounds,
        }
    }

    pub fn simplices<'a>(&'a self) -> FacetListIterator<'a, N> {
        FacetListIterator {
            ptr: self.cv.facets().ptr,
            pd: PhantomData,
        }
    }

    pub fn full_simplices<'a>(&'a self) -> FacetListIterator<'a, DimSum<N, U1>> {
        self.cv.facets()
    }

    pub fn vertices<'a>(&'a self) -> VertexListIterator<'a, N> {
        VertexListIterator {
            ptr: self.cv.vertices().ptr,
            pd: PhantomData
        }
    }

    pub fn full_vertices<'a>(&'a self) -> VertexListIterator<'a, DimSum<N, U1>> {
        self.cv.vertices()
    }

    /*
    pub fn find_best_simplex<'a>(&'a self, mut point: OVector<f64, N>) -> Option<Facet<'a, N>> {
        unsafe {
            let mut bestdist = 0.0;
            let mut is_outside = 0;
            let facet = qhull_sys::qh_findbestfacet(
                self.cv.qh.as_ptr(),
                point.as_mut_ptr(),
                1,// qh_ALL
                &mut bestdist,
                &mut is_outside
            );
            // if outside k
            Some(Facet {
                ptr: NonNull::new(facet).unwrap(),
                pd: PhantomData,
            })
        }
    }
    */

    // TODO: Add a parameter that provides starting point hint to make repeated lookups faster
    pub fn find_best_simplex<'a>(
        &'a self,
        point: OVector<f64, N>,
        solver_cache: &BarycentricSolverCache<N>,
        tolerance: Option<f64>,
        barycentric_coords: Option<&mut OVector<f64, DimSum<N, U1>>>,
        hint: Option<&mut Facet<'a, N>>,
    ) -> Option<Facet<'a, N>>
        where DefaultAllocator: Allocator<f64, DimSum<N, U1>, DimSum<N, U1>>,
              DefaultAllocator: Allocator<f64, DimSum<N, U1>>,
    {
        let tolerance = tolerance.unwrap_or(100.0 * f64::EPSILON);
        // let tolerance_sqrt = tolerance.sqrt();
        for ((n, max), min) in point.iter().zip(&self.max_bounds).zip(&self.min_bounds) {
            if *n > max + tolerance || *n < min - tolerance {
                return None
            }
        }

        let mut barycentric_coords_vec;
        let barycentric_coords = if let Some(coords) = barycentric_coords {
            coords
        } else {
            barycentric_coords_vec = OVector::from_element_generic(
                DimSum::<N, U1>::name(),
                U1::name(),
                0.0
            );
            &mut barycentric_coords_vec
        };

        let mut projected_point = OVector::from_iterator_generic(
            DimSum::<N, U1>::name(),
            U1::name(),
            point.iter().cloned().chain(iter::once(1.0)),
        );
        self.project_point(&mut projected_point);

        // Try to move to the approximately right location by repeatedly checking if any of the
        // current simplex's neighbors are closer to the point and moving to it
        let mut simplex = if let Some(hint) = &hint {
            Facet {
                ptr: hint.ptr,
                pd: PhantomData,
            }
        } else {
            self.full_simplices().filter(|s| !s.is_upper_delaunay()).next()?
        };
        let mut best_distance = simplex.plane_distance(&projected_point);
        let mut changed = true;
        while changed && best_distance <= 0.0 {
            changed = false;
            for neighbor in simplex.neighbors() {
                if neighbor.is_upper_delaunay() {
                    continue
                }
                let neighbor_distance = neighbor.plane_distance(&projected_point);
                if neighbor_distance > best_distance + tolerance * (1.0 + best_distance.abs()) {
                    // TODO: We should replace the iterator too? I guess it's a perf tweak
                    changed = true;
                    best_distance = neighbor_distance;
                    simplex = neighbor;
                }
            }
        }

        projected_point[N::dim()] = 1.0;
        'outer: for _cycle in 0..(solver_cache.0.len() / 4 + 1) {
            let mat = &solver_cache.0[simplex.id() as usize].as_ref().unwrap().mat;
            let mut inside = true;
            // Compute each barycentric coordinate one at a time
            let iter = mat.row_iter().zip(simplex.neighbors()).zip(barycentric_coords.iter_mut());
            for ((row, neighbor), bcoord) in iter {
                let coord = (row * &projected_point)[(0, 0)];
                if coord < -tolerance {
                    // Find the neighbor corresponding to the coord
                    if neighbor.is_upper_delaunay() {
                        if let Some(hint) = hint {
                            *hint = Facet {
                                ptr: simplex.ptr,
                                pd: PhantomData
                            };
                        }
                        return None;
                    }
                    simplex = neighbor;
                    continue 'outer;
                } else if !(coord <= 1.0 + tolerance) {
                    // Keep looking for a negative coordinate but note this isn't a match
                    inside = false;
                } else {
                    *bcoord = coord;
                }

            }

            if inside {
                // Success
                let f = Facet {
                    ptr: simplex.ptr,
                    pd: PhantomData,
                };
                if let Some(hint) = hint {
                    *hint = f;
                }
                return Some(f)
            }

            // We weren't inside the simplex and we didn't find a negative barycentric coordinate,
            // so we must have encountered a degenerate simplex. Fallback to bruteforce.
            break;
        }

        let f = self.find_best_simplex_bruteforce(point, solver_cache, tolerance, barycentric_coords);
        if let Some(f) = f {
            if let Some(hint) = hint {
                *hint = f;
            }
        }
        f
    }

    fn find_best_simplex_bruteforce<'a>(
        &'a self,
        point: OVector<f64, N>,
        solver_cache: &BarycentricSolverCache<N>,
        tolerance: f64,
        barycentric_coords: &mut OVector<f64, DimSum<N, U1>>,
    ) -> Option<Facet<'a, N>>
        where DefaultAllocator: Allocator<f64, DimSum<N, U1>, DimSum<N, U1>>,
              DefaultAllocator: Allocator<f64, DimSum<N, U1>>,
    {
        self.full_simplices()
            .filter(|simplex| !simplex.is_upper_delaunay())
            .find(|simplex| {
                // TODO: Special case degenerate simplices? I don't totally understand why SciPy
                //       has a special case for them
                let solver = solver_cache.0[simplex.id() as usize].as_ref().unwrap();
                solver.solve_mut(point.clone(), barycentric_coords);
                barycentric_coords.iter().all(|f| *f >= -tolerance && *f <= 1.0 + tolerance)
            })
            .map(|s| Facet {
                ptr: s.ptr,
                pd: PhantomData,
            })
    }

    pub fn compute_barycentric_solver_cache(&self) -> BarycentricSolverCache<N>
        where DefaultAllocator: Allocator<f64, DimSum<N, U1>, DimSum<N, U1>>,
    {
        let max_id = self.simplices()
            .filter(|s| !s.is_upper_delaunay())
            .map(|s| s.id() as usize)
            .max()
            .unwrap();
        let mut solvers = vec![None; max_id + 1];
        for simplex in self.simplices() {
            if simplex.is_upper_delaunay() {
                continue
            }
            let iter = simplex.vertices()
                .flat_map(|s| s.point().iter().copied().chain(iter::once(1.0)));
            // XXX Is there a way to do this without the DimName stuff?
            let mut mat = OMatrix::from_iterator_generic(
                DimSum::<N, U1>::name(),
                DimSum::<N, U1>::name(),
                iter
            );
            assert!(mat.try_inverse_mut());
            solvers[simplex.id() as usize] = Some(BarycentricSolver {
                mat,
            });
        }
        BarycentricSolverCache(solvers)
    }

    fn project_point(&self, point: &mut OVector<f64, DimSum<N, U1>>)
        where DefaultAllocator: Allocator<f64, DimSum<N, U1>>,
    {
        unsafe {
            qhull_sys::qh_setdelaunay(self.cv.qh.as_ptr(), N::dim() as i32 + 1, 1, point.as_mut_ptr());
        }
    }
}

#[derive(Clone, Debug)]
pub struct BarycentricSolverCache<N: DimName>(Vec<Option<BarycentricSolver<N>>>)
    where N: DimAdd<U1>,
          DefaultAllocator: Allocator<f64, DimSum<N, U1>, DimSum<N, U1>>;

impl<N: DimName> BarycentricSolverCache<N>
    where N: DimAdd<U1>,
          DefaultAllocator: Allocator<f64, DimSum<N, U1>, DimSum<N, U1>>,
{
    pub fn get(&self, simplex: &Facet<'_, N>) -> Option<&BarycentricSolver<N>> {
        self.0.get(simplex.id() as usize)?.as_ref()
    }
}

// Ugh, dimension generic code is sooooo ugly :(
#[derive(Clone, Debug)]
pub struct BarycentricSolver<N: DimName>
    where N: DimAdd<U1>,
          DefaultAllocator: Allocator<f64, DimSum<N, U1>, DimSum<N, U1>>
{
    mat: OMatrix<f64, DimSum<N, U1>, DimSum<N, U1>>,
}

impl<N: DimName> BarycentricSolver<N>
    where N: DimAdd<U1>,
          DefaultAllocator: Allocator<f64, DimSum<N, U1>, DimSum<N, U1>>,
{
    pub fn new(simplices: OMatrix<f64, N, DimSum<N, U1>>) -> Self
          where DefaultAllocator: Reallocator<f64, N, DimSum<N, U1>,
                                                   DimSum<N, U1>, DimSum<N, U1>>,
                DefaultAllocator: Allocator<f64, DimSum<N, U1>, N>,
    {
        let mut mat = simplices.insert_row(N::dim(), 1.0);
        assert!(mat.try_inverse_mut());
        BarycentricSolver {
            mat: mat,
        }
    }

    pub fn solve(&self, point: OVector<f64, N>) -> OVector<f64, DimSum<N, U1>>
        where DimSum<N, U1>: DimName,
              DefaultAllocator: Allocator<f64, N>,
              DefaultAllocator: Allocator<f64, DimSum<N, U1>>,
    {
        let vector = OVector::from_iterator_generic(
            DimSum::<N, U1>::name(),
            U1::name(),
            point.iter().copied().chain(iter::once(1.0))
        );
        (&self.mat) * vector
    }

    pub fn solve_mut(&self, point: OVector<f64, N>, out: &mut OVector<f64, DimSum<N, U1>>)
        where DimSum<N, U1>: DimName,
              DefaultAllocator: Allocator<f64, N>,
              DefaultAllocator: Allocator<f64, DimSum<N, U1>>,
    {
        let vector = OVector::from_iterator_generic(
            DimSum::<N, U1>::name(),
            U1::name(),
            point.iter().copied().chain(iter::once(1.0))
        );
        self.mat.mul_to(&vector, out);
    }
}


#[test]
fn test_cv_construction() {
    let points = [
        [1.0, 1.0],
        [2.0, 1.0],
        [1.0, 2.0],
        [2.0, 2.0],
        [1.5, 1.5],
        [1.5, 1.0],
    ];
    let cv = ConvexHull::from_arrays(&points[..]);
    let mut vertices = cv.vertices().map(|v| v.point()).collect::<Vec<_>>();
    vertices.sort_by(|a, b| a.partial_cmp(b).unwrap());
    assert_eq!(vertices, [
        [1.0, 1.0],
        [1.0, 2.0],
        [2.0, 1.0],
        [2.0, 2.0],
    ][..]);

    let mut facets = cv.facets()
        .map(|f| {
            let mut points = f.vertices()
                .map(|v| v.point())
                .collect::<Vec<_>>();
            points.sort_by(|a, b| a.partial_cmp(b).unwrap());
            points
        })
        .collect::<Vec<_>>();
    facets.sort_by(|a, b| a.partial_cmp(b).unwrap());
    assert_eq!(facets, [
        [[1.0, 1.0], [1.0, 2.0]],
        [[1.0, 1.0], [2.0, 1.0]],
        [[1.0, 2.0], [2.0, 2.0]],
        [[2.0, 1.0], [2.0, 2.0]],
    ][..]);
}

#[test]
fn test_delaunay_construction() {
    let points = [
        [1.0, 1.0].into(),
        [2.0, 1.0].into(),
        [1.0, 2.0].into(),
        [2.0, 2.0].into(),
    ];
    let tri = Delaunay::from_vectors(&points[..]);

    let mut simplices = tri.simplices()
        .filter(|s| !s.is_upper_delaunay())
        .map(|s| {
            let mut points = s.vertices()
                .map(|v| v.point())
                .collect::<Vec<_>>();
            points.sort_by(|a, b| a.partial_cmp(b).unwrap());
            points
        })
        .collect::<Vec<_>>();
    simplices.sort_by(|a, b| a.partial_cmp(b).unwrap());
    assert_eq!(simplices, [
        [[1.0, 1.0], [1.0, 2.0], [2.0, 2.0]],
        [[1.0, 1.0], [2.0, 1.0], [2.0, 2.0]],
    ][..]);
}

#[test]
fn test_delaunay_barycentric_solver_cache() {
    let points = [
        [1.0, 1.0],
        [2.0, 1.0],
        [1.0, 2.0],
        [2.0, 2.0],
    ];
    let tri = Delaunay::from_arrays(&points[..]);
    let _solver_cache = tri.compute_barycentric_solver_cache();
}

#[test]
fn test_delaunay_find_simplex_2d() {
    let points = [
        [1.0, 1.0],
        [2.0, 1.0],
        [1.0, 2.0],
        [2.0, 2.0],
    ];
    let tri = Delaunay::from_arrays(&points[..]);
    let solver_cache = tri.compute_barycentric_solver_cache();

    let mut barycentric_coords = OVector::from_element_generic(nalgebra::U3::name(), U1::name(), 0.0);

    let simplex = tri.find_best_simplex(
        [1.1, 1.9].into(),
        &solver_cache,
        None,
        Some(&mut barycentric_coords),
        None
    ).unwrap();
    let mut points = simplex.vertices().map(|v| v.point()).collect::<Vec<_>>();
    points.sort_by(|a, b| a.partial_cmp(b).unwrap());
    assert_eq!(points, [[1.0, 1.0], [1.0, 2.0], [2.0, 2.0]]);
    assert_eq!(
        barycentric_coords.iter().map(|f| (f * 10.0).round() / 10.0).collect::<Vec<_>>(),
        [0.1, 0.8, 0.1],
    );

    let simplex = tri.find_best_simplex(
        [1.9, 1.1].into(),
        &solver_cache,
        None,
        Some(&mut barycentric_coords),
        None
    ).unwrap();
    let mut points = simplex.vertices().map(|v| v.point()).collect::<Vec<_>>();
    points.sort_by(|a, b| a.partial_cmp(b).unwrap());
    assert_eq!(points, [
        [1.0, 1.0], [2.0, 1.0], [2.0, 2.0],
    ]);
    assert_eq!(
        barycentric_coords.iter().map(|f| (f * 10.0).round() / 10.0).collect::<Vec<_>>(),
        [0.1, 0.8, 0.1],
    );

    assert_eq!(None, tri.find_best_simplex([1.9, 2.1].into(), &solver_cache, None, None, None))
}

#[test]
fn test_delaunay_4d_simplices() {
    let points = [
        [3.0, 37.0, 3.0, 34.0],
        [8.0, 17.0, 4.0, 6.0],
        [29.0, 43.0, 32.0, 27.0],
        [41.0, 8.0, 44.0, 27.0],
        [44.0, 29.0, 33.0, 20.0],
        [44.0, 42.0, 38.0, 23.0],
        [48.0, 1.0, 9.0, 11.0],
    ];

    let tri = Delaunay::from_arrays(&points[..]);

    let correct_simplices = [
        [
            [3.0, 37.0, 3.0, 34.0],
            [8.0, 17.0, 4.0, 6.0],
            [29.0, 43.0, 32.0, 27.0],
            [41.0, 8.0, 44.0, 27.0],
            [44.0, 29.0, 33.0, 20.0],
        ],
        [
            [3.0, 37.0, 3.0, 34.0],
            [8.0, 17.0, 4.0, 6.0],
            [29.0, 43.0, 32.0, 27.0],
            [44.0, 29.0, 33.0, 20.0],
            [48.0, 1.0, 9.0, 11.0],
        ],
        [
            [3.0, 37.0, 3.0, 34.0],
            [8.0, 17.0, 4.0, 6.0],
            [29.0, 43.0, 32.0, 27.0],
            [44.0, 42.0, 38.0, 23.0],
            [48.0, 1.0, 9.0, 11.0],
        ],
        [
            [3.0, 37.0, 3.0, 34.0],
            [8.0, 17.0, 4.0, 6.0],
            [41.0, 8.0, 44.0, 27.0],
            [44.0, 29.0, 33.0, 20.0],
            [48.0, 1.0, 9.0, 11.0],
        ],
        [
            [3.0, 37.0, 3.0, 34.0],
            [29.0, 43.0, 32.0, 27.0],
            [41.0, 8.0, 44.0, 27.0],
            [44.0, 29.0, 33.0, 20.0],
            [44.0, 42.0, 38.0, 23.0],
        ],
        [
            [3.0, 37.0, 3.0, 34.0],
            [29.0, 43.0, 32.0, 27.0],
            [44.0, 29.0, 33.0, 20.0],
            [44.0, 42.0, 38.0, 23.0],
            [48.0, 1.0, 9.0, 11.0],
        ],
        [
            [3.0, 37.0, 3.0, 34.0],
            [41.0, 8.0, 44.0, 27.0],
            [44.0, 29.0, 33.0, 20.0],
            [44.0, 42.0, 38.0, 23.0],
            [48.0, 1.0, 9.0, 11.0],
        ],
        [
            [8.0, 17.0, 4.0, 6.0],
            [29.0, 43.0, 32.0, 27.0],
            [41.0, 8.0, 44.0, 27.0],
            [44.0, 29.0, 33.0, 20.0],
            [44.0, 42.0, 38.0, 23.0],
        ],
        [
            [8.0, 17.0, 4.0, 6.0],
            [29.0, 43.0, 32.0, 27.0],
            [44.0, 29.0, 33.0, 20.0],
            [44.0, 42.0, 38.0, 23.0],
            [48.0, 1.0, 9.0, 11.0],
        ],
    ];

    let mut simplices = tri.simplices()
        .filter(|s| !s.is_upper_delaunay())
        .map(|s| {
            let mut points = s.vertices()
                .map(|v| v.point().to_vec())
                .collect::<Vec<_>>();
            points.sort_by(|a, b| a.partial_cmp(b).unwrap());
            points
        })
        .collect::<Vec<_>>();
    simplices.sort_by(|a, b| a.partial_cmp(b).unwrap());
    assert_eq!(simplices, correct_simplices);
}

#[test]
fn test_delaunay_4d_find_simplex() {
    let points = [
        [3.0, 37.0, 3.0, 34.0],
        [8.0, 17.0, 4.0, 6.0],
        [29.0, 43.0, 32.0, 27.0],
        [41.0, 8.0, 44.0, 27.0],
        [44.0, 29.0, 33.0, 20.0],
        [44.0, 42.0, 38.0, 23.0],
        [48.0, 1.0, 9.0, 11.0],
    ];

    let tri = Delaunay::from_arrays(&points[..]);
    let solver_cache = tri.compute_barycentric_solver_cache();

    let mut barycentric_coords = OVector::from_element_generic(nalgebra::U5::name(), U1::name(), 0.0);

    // Precision issues...
    let trunc = |f: f64| (f * 10000000.0).round() / 10000000.0;

    let simplex = tri.find_best_simplex(
        [15.0, 29.0, 14.0, 24.0].into(),
        &solver_cache,
        None,
        Some(&mut barycentric_coords),
        None,
    ).unwrap();
    let mut points = simplex.vertices().map(|v| v.point()).collect::<Vec<_>>();
    points.sort_by(|a, b| a.partial_cmp(b).unwrap());
    assert_eq!(points, [
        [3.0, 37.0, 3.0, 34.0],
        [8.0, 17.0, 4.0, 6.0],
        [29.0, 43.0, 32.0, 27.0],
        [41.0, 8.0, 44.0, 27.0],
        [44.0, 29.0, 33.0, 20.0]
    ]);
    assert_eq!(barycentric_coords.iter().copied().map(trunc).collect::<Vec<_>>(), [
        trunc(0.06641458953962193),
        trunc(0.25979164147129463),
        trunc(0.10892298731140493),
        trunc(0.1476526962956015),
        trunc(0.41721808538207705)
    ]);

    let simplex = tri.find_best_simplex(
        [32.0, 19.0, 15.0, 18.0].into(),
        &solver_cache,
        None,
        Some(&mut barycentric_coords),
        None,
    ).unwrap();
    let mut points = simplex.vertices().map(|v| v.point()).collect::<Vec<_>>();
    points.sort_by(|a, b| a.partial_cmp(b).unwrap());
    assert_eq!(points, [
        [3.0, 37.0, 3.0, 34.0],
        [8.0, 17.0, 4.0, 6.0],
        [41.0, 8.0, 44.0, 27.0],
        [44.0, 29.0, 33.0, 20.0],
        [48.0, 1.0, 9.0, 11.0]
    ]);
    assert_eq!(barycentric_coords.iter().copied().map(trunc).collect::<Vec<_>>(), [
        trunc(0.29768201033578123),
        trunc(0.1383164403702304),
        trunc(0.02182084971165371),
        trunc(0.3394280685630884),
        trunc(0.20275263101924623)
    ]);

    let simplex = tri.find_best_simplex(
        [29.0, 17.0, 10.0, 17.0].into(),
        &solver_cache,
        None,
        Some(&mut barycentric_coords),
        None,
    ).unwrap();
    let mut points = simplex.vertices().map(|v| v.point()).collect::<Vec<_>>();
    points.sort_by(|a, b| a.partial_cmp(b).unwrap());
    assert_eq!(points, [
        [3.0, 37.0, 3.0, 34.0],
        [8.0, 17.0, 4.0, 6.0],
        [29.0, 43.0, 32.0, 27.0],
        [44.0, 42.0, 38.0, 23.0],
        [48.0, 1.0, 9.0, 11.0]
    ]);
    assert_eq!(barycentric_coords.iter().copied().map(trunc).collect::<Vec<_>>(), [
        trunc(0.005069878483865198),
        trunc(0.18000751093108713),
        trunc(0.13016711821668947),
        trunc(0.4779500523082701),
        trunc(0.20680544006008805)
    ]);

    let simplex = tri.find_best_simplex(
        [35.0, 35.0, 31.0, 25.0].into(),
        &solver_cache,
        None,
        Some(&mut barycentric_coords),
        None,
    ).unwrap();
    let mut points = simplex.vertices().map(|v| v.point()).collect::<Vec<_>>();
    points.sort_by(|a, b| a.partial_cmp(b).unwrap());
    assert_eq!(points, [
        [3.0, 37.0, 3.0, 34.0],
        [29.0, 43.0, 32.0, 27.0],
        [41.0, 8.0, 44.0, 27.0],
        [44.0, 29.0, 33.0, 20.0],
        [44.0, 42.0, 38.0, 23.0]
    ]);
    assert_eq!(barycentric_coords.iter().copied().map(trunc).collect::<Vec<_>>(), [
        trunc(0.23926991688134924),
        trunc(0.3536917318815922),
        trunc(0.09607252223788465),
        trunc(0.15530063675691475),
        trunc(0.15566519224225916)
    ]);

    let simplex = tri.find_best_simplex(
        [34.0, 27.0, 24.0, 19.0].into(),
        &solver_cache,
        None,
        Some(&mut barycentric_coords),
        None,
    ).unwrap();
    let mut points = simplex.vertices().map(|v| v.point()).collect::<Vec<_>>();
    points.sort_by(|a, b| a.partial_cmp(b).unwrap());
    assert_eq!(points, [
        [3.0, 37.0, 3.0, 34.0],
        [8.0, 17.0, 4.0, 6.0],
        [29.0, 43.0, 32.0, 27.0],
        [44.0, 29.0, 33.0, 20.0],
        [48.0, 1.0, 9.0, 11.0]
    ]);
    assert_eq!(barycentric_coords.iter().copied().map(trunc).collect::<Vec<_>>(), [
        trunc(0.5103273664105161),
        trunc(0.14791453826677897),
        trunc(0.16830275840625436),
        trunc(0.11024741260604956),
        trunc(0.0632079243104009)
    ]);

    let simplex = tri.find_best_simplex(
        [34.0, 34.0, 28.0, 22.0].into(),
        &solver_cache,
        None,
        Some(&mut barycentric_coords),
        None,
    ).unwrap();
    let mut points = simplex.vertices().map(|v| v.point()).collect::<Vec<_>>();
    points.sort_by(|a, b| a.partial_cmp(b).unwrap());
    assert_eq!(points, [
        [8.0, 17.0, 4.0, 6.0],
        [29.0, 43.0, 32.0, 27.0],
        [44.0, 29.0, 33.0, 20.0],
        [44.0, 42.0, 38.0, 23.0],
        [48.0, 1.0, 9.0, 11.0]
    ]);
    assert_eq!(barycentric_coords.iter().copied().map(trunc).collect::<Vec<_>>(), [
        trunc(0.0922839341817987),
        trunc(0.17098887054688694),
        trunc(0.07199531391447897),
        trunc(0.5298471697108473),
        trunc(0.13488471164598803)
    ]);

    let simplex = tri.find_best_simplex(
        [33.0, 31.0, 29.0, 20.0].into(),
        &solver_cache,
        None,
        Some(&mut barycentric_coords),
        None,
    ).unwrap();
    let mut points = simplex.vertices().map(|v| v.point()).collect::<Vec<_>>();
    points.sort_by(|a, b| a.partial_cmp(b).unwrap());
    assert_eq!(points, [
        [8.0, 17.0, 4.0, 6.0],
        [29.0, 43.0, 32.0, 27.0],
        [41.0, 8.0, 44.0, 27.0],
        [44.0, 29.0, 33.0, 20.0],
        [44.0, 42.0, 38.0, 23.0]
    ]);
    assert_eq!(barycentric_coords.iter().copied().map(trunc).collect::<Vec<_>>(), [
        trunc(0.23444866045762902),
        trunc(0.2519259514775205),
        trunc(0.20719788432792915),
        trunc(0.08796136598827176),
        trunc(0.21846613774864954)
    ]);

    let simplex = tri.find_best_simplex(
        [40.0, 30.0, 32.0, 23.0].into(),
        &solver_cache,
        None,
        Some(&mut barycentric_coords),
        None,
    ).unwrap();
    let mut points = simplex.vertices().map(|v| v.point()).collect::<Vec<_>>();
    points.sort_by(|a, b| a.partial_cmp(b).unwrap());
    assert_eq!(points, [
        [3.0, 37.0, 3.0, 34.0],
        [41.0, 8.0, 44.0, 27.0],
        [44.0, 29.0, 33.0, 20.0],
        [44.0, 42.0, 38.0, 23.0],
        [48.0, 1.0, 9.0, 11.0]
    ]);
    assert_eq!(barycentric_coords.iter().copied().map(trunc).collect::<Vec<_>>(), [
        trunc(0.19006624159575036),
        trunc(0.46741260823181996),
        trunc(0.1558290123303645),
        trunc(0.09159699307812597),
        trunc(0.09509514476393921)
    ]);

    let simplex = tri.find_best_simplex(
        [41.0, 30.0, 31.0, 21.0].into(),
        &solver_cache,
        None,
        Some(&mut barycentric_coords),
        None,
    ).unwrap();
    let mut points = simplex.vertices().map(|v| v.point()).collect::<Vec<_>>();
    points.sort_by(|a, b| a.partial_cmp(b).unwrap());
    assert_eq!(points, [
        [3.0, 37.0, 3.0, 34.0],
        [29.0, 43.0, 32.0, 27.0],
        [44.0, 29.0, 33.0, 20.0],
        [44.0, 42.0, 38.0, 23.0],
        [48.0, 1.0, 9.0, 11.0]
    ]);
    assert_eq!(barycentric_coords.iter().copied().map(trunc).collect::<Vec<_>>(), [
        trunc(0.8063201828480535),
        trunc(0.014011726125410462),
        trunc(0.10722448573983634),
        trunc(0.035079002285600236),
        trunc(0.03736460300109945)
    ]);
}

#[test]
fn test_plane_distance() {
    let points = [
        [1.0, 1.0],
        [2.0, 1.0],
        [1.0, 2.0],
        [2.0, 2.0],
    ];
    let cv = ConvexHull::from_arrays(&points[..]);
    let facet = cv.facets().next().unwrap();
    let mut point = [1.5, 2.5];
    let neighbor_distance = unsafe {
        let mut dist = 1.0;
        qhull_sys::qh_distplane(
            cv.qh.as_ptr(),
            point.as_mut_ptr(),
            facet.ptr.as_ptr(),
            &mut dist
        );
        dist
    };
    assert_eq!(neighbor_distance, facet.plane_distance(&point.into()));
}

// TODO: Test degenerate cases. Steal the test data from scipy
