/* automatically generated by rust-bindgen 0.58.1 */

#[repr(C)]
#[derive(Copy, Clone, Debug, Default, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct __BindgenBitfieldUnit<Storage> {
    storage: Storage,
}
impl<Storage> __BindgenBitfieldUnit<Storage> {
    #[inline]
    pub const fn new(storage: Storage) -> Self {
        Self { storage }
    }
}
impl<Storage> __BindgenBitfieldUnit<Storage>
where
    Storage: AsRef<[u8]> + AsMut<[u8]>,
{
    #[inline]
    pub fn get_bit(&self, index: usize) -> bool {
        debug_assert!(index / 8 < self.storage.as_ref().len());
        let byte_index = index / 8;
        let byte = self.storage.as_ref()[byte_index];
        let bit_index = if cfg!(target_endian = "big") {
            7 - (index % 8)
        } else {
            index % 8
        };
        let mask = 1 << bit_index;
        byte & mask == mask
    }
    #[inline]
    pub fn set_bit(&mut self, index: usize, val: bool) {
        debug_assert!(index / 8 < self.storage.as_ref().len());
        let byte_index = index / 8;
        let byte = &mut self.storage.as_mut()[byte_index];
        let bit_index = if cfg!(target_endian = "big") {
            7 - (index % 8)
        } else {
            index % 8
        };
        let mask = 1 << bit_index;
        if val {
            *byte |= mask;
        } else {
            *byte &= !mask;
        }
    }
    #[inline]
    pub fn get(&self, bit_offset: usize, bit_width: u8) -> u64 {
        debug_assert!(bit_width <= 64);
        debug_assert!(bit_offset / 8 < self.storage.as_ref().len());
        debug_assert!((bit_offset + (bit_width as usize)) / 8 <= self.storage.as_ref().len());
        let mut val = 0;
        for i in 0..(bit_width as usize) {
            if self.get_bit(i + bit_offset) {
                let index = if cfg!(target_endian = "big") {
                    bit_width as usize - 1 - i
                } else {
                    i
                };
                val |= 1 << index;
            }
        }
        val
    }
    #[inline]
    pub fn set(&mut self, bit_offset: usize, bit_width: u8, val: u64) {
        debug_assert!(bit_width <= 64);
        debug_assert!(bit_offset / 8 < self.storage.as_ref().len());
        debug_assert!((bit_offset + (bit_width as usize)) / 8 <= self.storage.as_ref().len());
        for i in 0..(bit_width as usize) {
            let mask = 1 << i;
            let val_bit_is_set = val & mask == mask;
            let index = if cfg!(target_endian = "big") {
                bit_width as usize - 1 - i
            } else {
                i
            };
            self.set_bit(index + bit_offset, val_bit_is_set);
        }
    }
}
pub type __int64_t = ::std::os::raw::c_longlong;
pub type __darwin_off_t = __int64_t;
pub type fpos_t = __darwin_off_t;
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct __sbuf {
    pub _base: *mut ::std::os::raw::c_uchar,
    pub _size: ::std::os::raw::c_int,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct __sFILEX {
    _unused: [u8; 0],
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct __sFILE {
    pub _p: *mut ::std::os::raw::c_uchar,
    pub _r: ::std::os::raw::c_int,
    pub _w: ::std::os::raw::c_int,
    pub _flags: ::std::os::raw::c_short,
    pub _file: ::std::os::raw::c_short,
    pub _bf: __sbuf,
    pub _lbfsize: ::std::os::raw::c_int,
    pub _cookie: *mut ::std::os::raw::c_void,
    pub _close: ::std::option::Option<
        unsafe extern "C" fn(arg1: *mut ::std::os::raw::c_void) -> ::std::os::raw::c_int,
    >,
    pub _read: ::std::option::Option<
        unsafe extern "C" fn(
            arg1: *mut ::std::os::raw::c_void,
            arg2: *mut ::std::os::raw::c_char,
            arg3: ::std::os::raw::c_int,
        ) -> ::std::os::raw::c_int,
    >,
    pub _seek: ::std::option::Option<
        unsafe extern "C" fn(
            arg1: *mut ::std::os::raw::c_void,
            arg2: fpos_t,
            arg3: ::std::os::raw::c_int,
        ) -> fpos_t,
    >,
    pub _write: ::std::option::Option<
        unsafe extern "C" fn(
            arg1: *mut ::std::os::raw::c_void,
            arg2: *const ::std::os::raw::c_char,
            arg3: ::std::os::raw::c_int,
        ) -> ::std::os::raw::c_int,
    >,
    pub _ub: __sbuf,
    pub _extra: *mut __sFILEX,
    pub _ur: ::std::os::raw::c_int,
    pub _ubuf: [::std::os::raw::c_uchar; 3usize],
    pub _nbuf: [::std::os::raw::c_uchar; 1usize],
    pub _lb: __sbuf,
    pub _blksize: ::std::os::raw::c_int,
    pub _offset: fpos_t,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct qhmemT {
    pub BUFsize: ::std::os::raw::c_int,
    pub BUFinit: ::std::os::raw::c_int,
    pub TABLEsize: ::std::os::raw::c_int,
    pub NUMsizes: ::std::os::raw::c_int,
    pub LASTsize: ::std::os::raw::c_int,
    pub ALIGNmask: ::std::os::raw::c_int,
    pub freelists: *mut *mut ::std::os::raw::c_void,
    pub sizetable: *mut ::std::os::raw::c_int,
    pub indextable: *mut ::std::os::raw::c_int,
    pub curbuffer: *mut ::std::os::raw::c_void,
    pub freemem: *mut ::std::os::raw::c_void,
    pub freesize: ::std::os::raw::c_int,
    pub tempstack: *mut setT,
    pub ferr: *mut FILE,
    pub IStracing: ::std::os::raw::c_int,
    pub cntquick: ::std::os::raw::c_int,
    pub cntshort: ::std::os::raw::c_int,
    pub cntlong: ::std::os::raw::c_int,
    pub freeshort: ::std::os::raw::c_int,
    pub freelong: ::std::os::raw::c_int,
    pub totbuffer: ::std::os::raw::c_int,
    pub totdropped: ::std::os::raw::c_int,
    pub totfree: ::std::os::raw::c_int,
    pub totlong: ::std::os::raw::c_int,
    pub maxlong: ::std::os::raw::c_int,
    pub totshort: ::std::os::raw::c_int,
    pub totunused: ::std::os::raw::c_int,
    pub cntlarger: ::std::os::raw::c_int,
    pub totlarger: ::std::os::raw::c_int,
}
extern "C" {
    pub fn qh_memfreeshort(
        qh: *mut qhT,
        curlong: *mut ::std::os::raw::c_int,
        totlong: *mut ::std::os::raw::c_int,
    );
}
#[repr(C)]
#[derive(Copy, Clone)]
pub union setelemT {
    pub p: *mut ::std::os::raw::c_void,
    pub i: ::std::os::raw::c_int,
}
#[repr(C)]
#[derive(Copy, Clone)]
pub struct setT {
    pub maxsize: ::std::os::raw::c_int,
    pub e: [setelemT; 1usize],
}
extern "C" {
    pub fn qh_setappend(qh: *mut qhT, setp: *mut *mut setT, elem: *mut ::std::os::raw::c_void);
}
extern "C" {
    pub fn qh_setsize(qh: *mut qhT, set: *mut setT) -> ::std::os::raw::c_int;
}
#[repr(C)]
#[derive(Copy, Clone)]
pub union intrealT {
    pub i: ::std::os::raw::c_int,
    pub r: f64,
}
#[repr(C)]
#[derive(Copy, Clone)]
pub struct qhstatT {
    pub stats: [intrealT; 249usize],
    pub id: [::std::os::raw::c_uchar; 259usize],
    pub doc: [*const ::std::os::raw::c_char; 249usize],
    pub count: [::std::os::raw::c_short; 249usize],
    pub type_: [::std::os::raw::c_char; 249usize],
    pub printed: [::std::os::raw::c_char; 249usize],
    pub init: [intrealT; 9usize],
    pub next: ::std::os::raw::c_int,
    pub precision: ::std::os::raw::c_int,
    pub vridges: ::std::os::raw::c_int,
    pub tempi: ::std::os::raw::c_int,
    pub tempr: f64,
}
pub const qh_CENTER_qh_ASnone: qh_CENTER = 0;
pub const qh_CENTER_qh_ASvoronoi: qh_CENTER = 1;
pub const qh_CENTER_qh_AScentrum: qh_CENTER = 2;
pub type qh_CENTER = ::std::os::raw::c_uint;
pub const qh_PRINT_qh_PRINTnone: qh_PRINT = 0;
pub const qh_PRINT_qh_PRINTarea: qh_PRINT = 1;
pub const qh_PRINT_qh_PRINTaverage: qh_PRINT = 2;
pub const qh_PRINT_qh_PRINTcoplanars: qh_PRINT = 3;
pub const qh_PRINT_qh_PRINTcentrums: qh_PRINT = 4;
pub const qh_PRINT_qh_PRINTfacets: qh_PRINT = 5;
pub const qh_PRINT_qh_PRINTfacets_xridge: qh_PRINT = 6;
pub const qh_PRINT_qh_PRINTgeom: qh_PRINT = 7;
pub const qh_PRINT_qh_PRINTids: qh_PRINT = 8;
pub const qh_PRINT_qh_PRINTinner: qh_PRINT = 9;
pub const qh_PRINT_qh_PRINTneighbors: qh_PRINT = 10;
pub const qh_PRINT_qh_PRINTnormals: qh_PRINT = 11;
pub const qh_PRINT_qh_PRINTouter: qh_PRINT = 12;
pub const qh_PRINT_qh_PRINTmaple: qh_PRINT = 13;
pub const qh_PRINT_qh_PRINTincidences: qh_PRINT = 14;
pub const qh_PRINT_qh_PRINTmathematica: qh_PRINT = 15;
pub const qh_PRINT_qh_PRINTmerges: qh_PRINT = 16;
pub const qh_PRINT_qh_PRINToff: qh_PRINT = 17;
pub const qh_PRINT_qh_PRINToptions: qh_PRINT = 18;
pub const qh_PRINT_qh_PRINTpointintersect: qh_PRINT = 19;
pub const qh_PRINT_qh_PRINTpointnearest: qh_PRINT = 20;
pub const qh_PRINT_qh_PRINTpoints: qh_PRINT = 21;
pub const qh_PRINT_qh_PRINTqhull: qh_PRINT = 22;
pub const qh_PRINT_qh_PRINTsize: qh_PRINT = 23;
pub const qh_PRINT_qh_PRINTsummary: qh_PRINT = 24;
pub const qh_PRINT_qh_PRINTtriangles: qh_PRINT = 25;
pub const qh_PRINT_qh_PRINTvertices: qh_PRINT = 26;
pub const qh_PRINT_qh_PRINTvneighbors: qh_PRINT = 27;
pub const qh_PRINT_qh_PRINTextremes: qh_PRINT = 28;
pub const qh_PRINT_qh_PRINTEND: qh_PRINT = 29;
pub type qh_PRINT = ::std::os::raw::c_uint;
#[repr(C)]
#[derive(Copy, Clone)]
pub struct facetT {
    pub furthestdist: f64,
    pub maxoutside: f64,
    pub offset: f64,
    pub normal: *mut f64,
    pub f: facetT__bindgen_ty_1,
    pub center: *mut f64,
    pub previous: *mut facetT,
    pub next: *mut facetT,
    pub vertices: *mut setT,
    pub ridges: *mut setT,
    pub neighbors: *mut setT,
    pub outsideset: *mut setT,
    pub coplanarset: *mut setT,
    pub visitid: ::std::os::raw::c_uint,
    pub id: ::std::os::raw::c_uint,
    pub _bitfield_align_1: [u16; 0],
    pub _bitfield_1: __BindgenBitfieldUnit<[u8; 4usize]>,
    pub __bindgen_padding_0: u32,
}
#[repr(C)]
#[derive(Copy, Clone)]
pub union facetT__bindgen_ty_1 {
    pub area: f64,
    pub replace: *mut facetT,
    pub samecycle: *mut facetT,
    pub newcycle: *mut facetT,
    pub trivisible: *mut facetT,
    pub triowner: *mut facetT,
}
impl facetT {
    #[inline]
    pub fn nummerge(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(0usize, 9u8) as u32) }
    }
    #[inline]
    pub fn set_nummerge(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(0usize, 9u8, val as u64)
        }
    }
    #[inline]
    pub fn tricoplanar(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(9usize, 1u8) as u32) }
    }
    #[inline]
    pub fn set_tricoplanar(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(9usize, 1u8, val as u64)
        }
    }
    #[inline]
    pub fn newfacet(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(10usize, 1u8) as u32) }
    }
    #[inline]
    pub fn set_newfacet(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(10usize, 1u8, val as u64)
        }
    }
    #[inline]
    pub fn visible(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(11usize, 1u8) as u32) }
    }
    #[inline]
    pub fn set_visible(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(11usize, 1u8, val as u64)
        }
    }
    #[inline]
    pub fn toporient(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(12usize, 1u8) as u32) }
    }
    #[inline]
    pub fn set_toporient(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(12usize, 1u8, val as u64)
        }
    }
    #[inline]
    pub fn simplicial(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(13usize, 1u8) as u32) }
    }
    #[inline]
    pub fn set_simplicial(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(13usize, 1u8, val as u64)
        }
    }
    #[inline]
    pub fn seen(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(14usize, 1u8) as u32) }
    }
    #[inline]
    pub fn set_seen(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(14usize, 1u8, val as u64)
        }
    }
    #[inline]
    pub fn seen2(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(15usize, 1u8) as u32) }
    }
    #[inline]
    pub fn set_seen2(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(15usize, 1u8, val as u64)
        }
    }
    #[inline]
    pub fn flipped(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(16usize, 1u8) as u32) }
    }
    #[inline]
    pub fn set_flipped(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(16usize, 1u8, val as u64)
        }
    }
    #[inline]
    pub fn upperdelaunay(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(17usize, 1u8) as u32) }
    }
    #[inline]
    pub fn set_upperdelaunay(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(17usize, 1u8, val as u64)
        }
    }
    #[inline]
    pub fn notfurthest(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(18usize, 1u8) as u32) }
    }
    #[inline]
    pub fn set_notfurthest(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(18usize, 1u8, val as u64)
        }
    }
    #[inline]
    pub fn good(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(19usize, 1u8) as u32) }
    }
    #[inline]
    pub fn set_good(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(19usize, 1u8, val as u64)
        }
    }
    #[inline]
    pub fn isarea(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(20usize, 1u8) as u32) }
    }
    #[inline]
    pub fn set_isarea(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(20usize, 1u8, val as u64)
        }
    }
    #[inline]
    pub fn dupridge(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(21usize, 1u8) as u32) }
    }
    #[inline]
    pub fn set_dupridge(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(21usize, 1u8, val as u64)
        }
    }
    #[inline]
    pub fn mergeridge(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(22usize, 1u8) as u32) }
    }
    #[inline]
    pub fn set_mergeridge(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(22usize, 1u8, val as u64)
        }
    }
    #[inline]
    pub fn mergeridge2(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(23usize, 1u8) as u32) }
    }
    #[inline]
    pub fn set_mergeridge2(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(23usize, 1u8, val as u64)
        }
    }
    #[inline]
    pub fn coplanarhorizon(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(24usize, 1u8) as u32) }
    }
    #[inline]
    pub fn set_coplanarhorizon(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(24usize, 1u8, val as u64)
        }
    }
    #[inline]
    pub fn mergehorizon(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(25usize, 1u8) as u32) }
    }
    #[inline]
    pub fn set_mergehorizon(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(25usize, 1u8, val as u64)
        }
    }
    #[inline]
    pub fn cycledone(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(26usize, 1u8) as u32) }
    }
    #[inline]
    pub fn set_cycledone(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(26usize, 1u8, val as u64)
        }
    }
    #[inline]
    pub fn tested(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(27usize, 1u8) as u32) }
    }
    #[inline]
    pub fn set_tested(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(27usize, 1u8, val as u64)
        }
    }
    #[inline]
    pub fn keepcentrum(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(28usize, 1u8) as u32) }
    }
    #[inline]
    pub fn set_keepcentrum(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(28usize, 1u8, val as u64)
        }
    }
    #[inline]
    pub fn newmerge(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(29usize, 1u8) as u32) }
    }
    #[inline]
    pub fn set_newmerge(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(29usize, 1u8, val as u64)
        }
    }
    #[inline]
    pub fn degenerate(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(30usize, 1u8) as u32) }
    }
    #[inline]
    pub fn set_degenerate(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(30usize, 1u8, val as u64)
        }
    }
    #[inline]
    pub fn redundant(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(31usize, 1u8) as u32) }
    }
    #[inline]
    pub fn set_redundant(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(31usize, 1u8, val as u64)
        }
    }
    #[inline]
    pub fn new_bitfield_1(
        nummerge: ::std::os::raw::c_uint,
        tricoplanar: ::std::os::raw::c_uint,
        newfacet: ::std::os::raw::c_uint,
        visible: ::std::os::raw::c_uint,
        toporient: ::std::os::raw::c_uint,
        simplicial: ::std::os::raw::c_uint,
        seen: ::std::os::raw::c_uint,
        seen2: ::std::os::raw::c_uint,
        flipped: ::std::os::raw::c_uint,
        upperdelaunay: ::std::os::raw::c_uint,
        notfurthest: ::std::os::raw::c_uint,
        good: ::std::os::raw::c_uint,
        isarea: ::std::os::raw::c_uint,
        dupridge: ::std::os::raw::c_uint,
        mergeridge: ::std::os::raw::c_uint,
        mergeridge2: ::std::os::raw::c_uint,
        coplanarhorizon: ::std::os::raw::c_uint,
        mergehorizon: ::std::os::raw::c_uint,
        cycledone: ::std::os::raw::c_uint,
        tested: ::std::os::raw::c_uint,
        keepcentrum: ::std::os::raw::c_uint,
        newmerge: ::std::os::raw::c_uint,
        degenerate: ::std::os::raw::c_uint,
        redundant: ::std::os::raw::c_uint,
    ) -> __BindgenBitfieldUnit<[u8; 4usize]> {
        let mut __bindgen_bitfield_unit: __BindgenBitfieldUnit<[u8; 4usize]> = Default::default();
        __bindgen_bitfield_unit.set(0usize, 9u8, {
            let nummerge: u32 = unsafe { ::std::mem::transmute(nummerge) };
            nummerge as u64
        });
        __bindgen_bitfield_unit.set(9usize, 1u8, {
            let tricoplanar: u32 = unsafe { ::std::mem::transmute(tricoplanar) };
            tricoplanar as u64
        });
        __bindgen_bitfield_unit.set(10usize, 1u8, {
            let newfacet: u32 = unsafe { ::std::mem::transmute(newfacet) };
            newfacet as u64
        });
        __bindgen_bitfield_unit.set(11usize, 1u8, {
            let visible: u32 = unsafe { ::std::mem::transmute(visible) };
            visible as u64
        });
        __bindgen_bitfield_unit.set(12usize, 1u8, {
            let toporient: u32 = unsafe { ::std::mem::transmute(toporient) };
            toporient as u64
        });
        __bindgen_bitfield_unit.set(13usize, 1u8, {
            let simplicial: u32 = unsafe { ::std::mem::transmute(simplicial) };
            simplicial as u64
        });
        __bindgen_bitfield_unit.set(14usize, 1u8, {
            let seen: u32 = unsafe { ::std::mem::transmute(seen) };
            seen as u64
        });
        __bindgen_bitfield_unit.set(15usize, 1u8, {
            let seen2: u32 = unsafe { ::std::mem::transmute(seen2) };
            seen2 as u64
        });
        __bindgen_bitfield_unit.set(16usize, 1u8, {
            let flipped: u32 = unsafe { ::std::mem::transmute(flipped) };
            flipped as u64
        });
        __bindgen_bitfield_unit.set(17usize, 1u8, {
            let upperdelaunay: u32 = unsafe { ::std::mem::transmute(upperdelaunay) };
            upperdelaunay as u64
        });
        __bindgen_bitfield_unit.set(18usize, 1u8, {
            let notfurthest: u32 = unsafe { ::std::mem::transmute(notfurthest) };
            notfurthest as u64
        });
        __bindgen_bitfield_unit.set(19usize, 1u8, {
            let good: u32 = unsafe { ::std::mem::transmute(good) };
            good as u64
        });
        __bindgen_bitfield_unit.set(20usize, 1u8, {
            let isarea: u32 = unsafe { ::std::mem::transmute(isarea) };
            isarea as u64
        });
        __bindgen_bitfield_unit.set(21usize, 1u8, {
            let dupridge: u32 = unsafe { ::std::mem::transmute(dupridge) };
            dupridge as u64
        });
        __bindgen_bitfield_unit.set(22usize, 1u8, {
            let mergeridge: u32 = unsafe { ::std::mem::transmute(mergeridge) };
            mergeridge as u64
        });
        __bindgen_bitfield_unit.set(23usize, 1u8, {
            let mergeridge2: u32 = unsafe { ::std::mem::transmute(mergeridge2) };
            mergeridge2 as u64
        });
        __bindgen_bitfield_unit.set(24usize, 1u8, {
            let coplanarhorizon: u32 = unsafe { ::std::mem::transmute(coplanarhorizon) };
            coplanarhorizon as u64
        });
        __bindgen_bitfield_unit.set(25usize, 1u8, {
            let mergehorizon: u32 = unsafe { ::std::mem::transmute(mergehorizon) };
            mergehorizon as u64
        });
        __bindgen_bitfield_unit.set(26usize, 1u8, {
            let cycledone: u32 = unsafe { ::std::mem::transmute(cycledone) };
            cycledone as u64
        });
        __bindgen_bitfield_unit.set(27usize, 1u8, {
            let tested: u32 = unsafe { ::std::mem::transmute(tested) };
            tested as u64
        });
        __bindgen_bitfield_unit.set(28usize, 1u8, {
            let keepcentrum: u32 = unsafe { ::std::mem::transmute(keepcentrum) };
            keepcentrum as u64
        });
        __bindgen_bitfield_unit.set(29usize, 1u8, {
            let newmerge: u32 = unsafe { ::std::mem::transmute(newmerge) };
            newmerge as u64
        });
        __bindgen_bitfield_unit.set(30usize, 1u8, {
            let degenerate: u32 = unsafe { ::std::mem::transmute(degenerate) };
            degenerate as u64
        });
        __bindgen_bitfield_unit.set(31usize, 1u8, {
            let redundant: u32 = unsafe { ::std::mem::transmute(redundant) };
            redundant as u64
        });
        __bindgen_bitfield_unit
    }
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct ridgeT {
    pub vertices: *mut setT,
    pub top: *mut facetT,
    pub bottom: *mut facetT,
    pub id: ::std::os::raw::c_uint,
    pub _bitfield_align_1: [u8; 0],
    pub _bitfield_1: __BindgenBitfieldUnit<[u8; 1usize]>,
    pub __bindgen_padding_0: [u8; 3usize],
}
impl ridgeT {
    #[inline]
    pub fn seen(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(0usize, 1u8) as u32) }
    }
    #[inline]
    pub fn set_seen(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(0usize, 1u8, val as u64)
        }
    }
    #[inline]
    pub fn tested(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(1usize, 1u8) as u32) }
    }
    #[inline]
    pub fn set_tested(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(1usize, 1u8, val as u64)
        }
    }
    #[inline]
    pub fn nonconvex(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(2usize, 1u8) as u32) }
    }
    #[inline]
    pub fn set_nonconvex(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(2usize, 1u8, val as u64)
        }
    }
    #[inline]
    pub fn mergevertex(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(3usize, 1u8) as u32) }
    }
    #[inline]
    pub fn set_mergevertex(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(3usize, 1u8, val as u64)
        }
    }
    #[inline]
    pub fn mergevertex2(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(4usize, 1u8) as u32) }
    }
    #[inline]
    pub fn set_mergevertex2(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(4usize, 1u8, val as u64)
        }
    }
    #[inline]
    pub fn simplicialtop(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(5usize, 1u8) as u32) }
    }
    #[inline]
    pub fn set_simplicialtop(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(5usize, 1u8, val as u64)
        }
    }
    #[inline]
    pub fn simplicialbot(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(6usize, 1u8) as u32) }
    }
    #[inline]
    pub fn set_simplicialbot(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(6usize, 1u8, val as u64)
        }
    }
    #[inline]
    pub fn new_bitfield_1(
        seen: ::std::os::raw::c_uint,
        tested: ::std::os::raw::c_uint,
        nonconvex: ::std::os::raw::c_uint,
        mergevertex: ::std::os::raw::c_uint,
        mergevertex2: ::std::os::raw::c_uint,
        simplicialtop: ::std::os::raw::c_uint,
        simplicialbot: ::std::os::raw::c_uint,
    ) -> __BindgenBitfieldUnit<[u8; 1usize]> {
        let mut __bindgen_bitfield_unit: __BindgenBitfieldUnit<[u8; 1usize]> = Default::default();
        __bindgen_bitfield_unit.set(0usize, 1u8, {
            let seen: u32 = unsafe { ::std::mem::transmute(seen) };
            seen as u64
        });
        __bindgen_bitfield_unit.set(1usize, 1u8, {
            let tested: u32 = unsafe { ::std::mem::transmute(tested) };
            tested as u64
        });
        __bindgen_bitfield_unit.set(2usize, 1u8, {
            let nonconvex: u32 = unsafe { ::std::mem::transmute(nonconvex) };
            nonconvex as u64
        });
        __bindgen_bitfield_unit.set(3usize, 1u8, {
            let mergevertex: u32 = unsafe { ::std::mem::transmute(mergevertex) };
            mergevertex as u64
        });
        __bindgen_bitfield_unit.set(4usize, 1u8, {
            let mergevertex2: u32 = unsafe { ::std::mem::transmute(mergevertex2) };
            mergevertex2 as u64
        });
        __bindgen_bitfield_unit.set(5usize, 1u8, {
            let simplicialtop: u32 = unsafe { ::std::mem::transmute(simplicialtop) };
            simplicialtop as u64
        });
        __bindgen_bitfield_unit.set(6usize, 1u8, {
            let simplicialbot: u32 = unsafe { ::std::mem::transmute(simplicialbot) };
            simplicialbot as u64
        });
        __bindgen_bitfield_unit
    }
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct vertexT {
    pub next: *mut vertexT,
    pub previous: *mut vertexT,
    pub point: *mut f64,
    pub neighbors: *mut setT,
    pub id: ::std::os::raw::c_uint,
    pub visitid: ::std::os::raw::c_uint,
    pub _bitfield_align_1: [u8; 0],
    pub _bitfield_1: __BindgenBitfieldUnit<[u8; 1usize]>,
    pub __bindgen_padding_0: [u8; 7usize],
}
impl vertexT {
    #[inline]
    pub fn seen(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(0usize, 1u8) as u32) }
    }
    #[inline]
    pub fn set_seen(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(0usize, 1u8, val as u64)
        }
    }
    #[inline]
    pub fn seen2(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(1usize, 1u8) as u32) }
    }
    #[inline]
    pub fn set_seen2(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(1usize, 1u8, val as u64)
        }
    }
    #[inline]
    pub fn deleted(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(2usize, 1u8) as u32) }
    }
    #[inline]
    pub fn set_deleted(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(2usize, 1u8, val as u64)
        }
    }
    #[inline]
    pub fn delridge(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(3usize, 1u8) as u32) }
    }
    #[inline]
    pub fn set_delridge(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(3usize, 1u8, val as u64)
        }
    }
    #[inline]
    pub fn newfacet(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(4usize, 1u8) as u32) }
    }
    #[inline]
    pub fn set_newfacet(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(4usize, 1u8, val as u64)
        }
    }
    #[inline]
    pub fn partitioned(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(5usize, 1u8) as u32) }
    }
    #[inline]
    pub fn set_partitioned(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(5usize, 1u8, val as u64)
        }
    }
    #[inline]
    pub fn new_bitfield_1(
        seen: ::std::os::raw::c_uint,
        seen2: ::std::os::raw::c_uint,
        deleted: ::std::os::raw::c_uint,
        delridge: ::std::os::raw::c_uint,
        newfacet: ::std::os::raw::c_uint,
        partitioned: ::std::os::raw::c_uint,
    ) -> __BindgenBitfieldUnit<[u8; 1usize]> {
        let mut __bindgen_bitfield_unit: __BindgenBitfieldUnit<[u8; 1usize]> = Default::default();
        __bindgen_bitfield_unit.set(0usize, 1u8, {
            let seen: u32 = unsafe { ::std::mem::transmute(seen) };
            seen as u64
        });
        __bindgen_bitfield_unit.set(1usize, 1u8, {
            let seen2: u32 = unsafe { ::std::mem::transmute(seen2) };
            seen2 as u64
        });
        __bindgen_bitfield_unit.set(2usize, 1u8, {
            let deleted: u32 = unsafe { ::std::mem::transmute(deleted) };
            deleted as u64
        });
        __bindgen_bitfield_unit.set(3usize, 1u8, {
            let delridge: u32 = unsafe { ::std::mem::transmute(delridge) };
            delridge as u64
        });
        __bindgen_bitfield_unit.set(4usize, 1u8, {
            let newfacet: u32 = unsafe { ::std::mem::transmute(newfacet) };
            newfacet as u64
        });
        __bindgen_bitfield_unit.set(5usize, 1u8, {
            let partitioned: u32 = unsafe { ::std::mem::transmute(partitioned) };
            partitioned as u64
        });
        __bindgen_bitfield_unit
    }
}
extern "C" {
    pub fn qh_qhull(qh: *mut qhT);
}
extern "C" {
    pub fn qh_addpoint(
        qh: *mut qhT,
        furthest: *mut f64,
        facet: *mut facetT,
        checkdist: ::std::os::raw::c_uint,
    ) -> ::std::os::raw::c_uint;
}
extern "C" {
    pub fn qh_setdelaunay(
        qh: *mut qhT,
        dim: ::std::os::raw::c_int,
        count: ::std::os::raw::c_int,
        points: *mut f64,
    );
}
extern "C" {
    pub fn qh_sethalfspace_all(
        qh: *mut qhT,
        dim: ::std::os::raw::c_int,
        count: ::std::os::raw::c_int,
        halfspaces: *mut f64,
        feasible: *mut f64,
    ) -> *mut f64;
}
extern "C" {
    pub fn qh_checkflags(
        qh: *mut qhT,
        command: *mut ::std::os::raw::c_char,
        hiddenflags: *mut ::std::os::raw::c_char,
    );
}
extern "C" {
    pub fn qh_freeqhull(qh: *mut qhT, allmem: ::std::os::raw::c_uint);
}
extern "C" {
    pub fn qh_init_A(
        qh: *mut qhT,
        infile: *mut FILE,
        outfile: *mut FILE,
        errfile: *mut FILE,
        argc: ::std::os::raw::c_int,
        argv: *mut *mut ::std::os::raw::c_char,
    );
}
extern "C" {
    pub fn qh_init_B(
        qh: *mut qhT,
        points: *mut f64,
        numpoints: ::std::os::raw::c_int,
        dim: ::std::os::raw::c_int,
        ismalloc: ::std::os::raw::c_uint,
    );
}
extern "C" {
    pub fn qh_initflags(qh: *mut qhT, command: *mut ::std::os::raw::c_char);
}
extern "C" {
    pub fn qh_option(
        qh: *mut qhT,
        option: *const ::std::os::raw::c_char,
        i: *mut ::std::os::raw::c_int,
        r: *mut f64,
    );
}
extern "C" {
    pub fn qh_zero(qh: *mut qhT, errfile: *mut FILE);
}
extern "C" {
    pub fn qh_produce_output(qh: *mut qhT);
}
extern "C" {
    pub fn qh_readpoints(
        qh: *mut qhT,
        numpoints: *mut ::std::os::raw::c_int,
        dimension: *mut ::std::os::raw::c_int,
        ismalloc: *mut ::std::os::raw::c_uint,
    ) -> *mut f64;
}
extern "C" {
    pub fn qh_check_output(qh: *mut qhT);
}
extern "C" {
    pub fn qh_findbestfacet(
        qh: *mut qhT,
        point: *mut f64,
        bestoutside: ::std::os::raw::c_uint,
        bestdist: *mut f64,
        isoutside: *mut ::std::os::raw::c_uint,
    ) -> *mut facetT;
}
extern "C" {
    pub fn qh_nearvertex(
        qh: *mut qhT,
        facet: *mut facetT,
        point: *mut f64,
        bestdistp: *mut f64,
    ) -> *mut vertexT;
}
extern "C" {
    pub fn qh_pointid(qh: *mut qhT, point: *mut f64) -> ::std::os::raw::c_int;
}
extern "C" {
    pub fn qh_pointvertex(qh: *mut qhT) -> *mut setT;
}
extern "C" {
    pub fn qh_triangulate(qh: *mut qhT);
}
extern "C" {
    pub fn qh_distplane(qh: *mut qhT, point: *mut f64, facet: *mut facetT, dist: *mut f64);
}
extern "C" {
    pub fn qh_facetcenter(qh: *mut qhT, vertices: *mut setT) -> *mut f64;
}
extern "C" {
    pub fn qh_getarea(qh: *mut qhT, facetlist: *mut facetT);
}
pub const qh_RIDGE_qh_RIDGEall: qh_RIDGE = 0;
pub const qh_RIDGE_qh_RIDGEinner: qh_RIDGE = 1;
pub const qh_RIDGE_qh_RIDGEouter: qh_RIDGE = 2;
pub type qh_RIDGE = ::std::os::raw::c_uint;
pub type printvridgeT = ::std::option::Option<
    unsafe extern "C" fn(
        qh: *mut qhT,
        fp: *mut FILE,
        vertex: *mut vertexT,
        vertexA: *mut vertexT,
        centers: *mut setT,
        unbounded: ::std::os::raw::c_uint,
    ),
>;
extern "C" {
    pub fn qh_compare_facetvisit(
        p1: *const ::std::os::raw::c_void,
        p2: *const ::std::os::raw::c_void,
    ) -> ::std::os::raw::c_int;
}
extern "C" {
    pub fn qh_eachvoronoi_all(
        qh: *mut qhT,
        fp: *mut FILE,
        printvridge: printvridgeT,
        isUpper: ::std::os::raw::c_uint,
        innerouter: qh_RIDGE,
        inorder: ::std::os::raw::c_uint,
    ) -> ::std::os::raw::c_int;
}
extern "C" {
    pub fn qh_order_vertexneighbors(qh: *mut qhT, vertex: *mut vertexT);
}
extern "C" {
    pub fn qh_check_maxout(qh: *mut qhT);
}
extern "C" {
    pub fn qh_checkpolygon(qh: *mut qhT, facetlist: *mut facetT);
}
extern "C" {
    pub fn qh_findgood_all(qh: *mut qhT, facetlist: *mut facetT);
}
extern "C" {
    pub fn qh_appendprint(qh: *mut qhT, format: qh_PRINT);
}
extern "C" {
    pub fn rqh_malloc() -> *mut qhT;
}
extern "C" {
    pub fn rqh_get_NOerrexit(qh: *mut qhT) -> *mut ::std::os::raw::c_uint;
}
extern "C" {
    pub fn rqh_get_facet_list(qh: *mut qhT) -> *mut *mut facetT;
}
extern "C" {
    pub fn rqh_get_vertex_list(qh: *mut qhT) -> *mut *mut vertexT;
}
extern "C" {
    pub fn rqh_get_PROJECTdelaunay(qh: *mut qhT) -> *mut ::std::os::raw::c_uint;
}
extern "C" {
    pub fn rqh_stdin() -> *mut FILE;
}
extern "C" {
    pub fn rqh_stdout() -> *mut FILE;
}
extern "C" {
    pub fn rqh_stderr() -> *mut FILE;
}
