#![feature(maybe_uninit_uninit_array)]
#![feature(maybe_uninit_slice)]
#![feature(maybe_uninit_extra)]
#![feature(option_result_unwrap_unchecked)]

mod error;

use std::convert::identity as id;
use std::default::Default;
use std::fmt::{self, Debug};
use std::mem::MaybeUninit;
use std::ops::{Deref, DerefMut, Drop};

use error::*;

pub struct VecArray<T, const N: usize> {
    buf: [MaybeUninit<T>; N],
    len: usize,
}

impl<T, const N: usize> VecArray<T, N> {
    pub fn new() -> Self {
        VecArray {
            buf: MaybeUninit::uninit_array(),
            len: 0,
        }
    }

    pub fn new_sorted() -> Sorted<Self> {
        Sorted::new_by_key(VecArray::new(), |_| unreachable!("VecArray is empty"))
    }

    #[inline(always)]
    pub fn len(&self) -> usize {
        self.len
    }

    pub fn as_slice(&self) -> &[T] {
        // SAFETY: indexes from 0 to len are garanteed to be valid.
        unsafe { MaybeUninit::slice_assume_init_ref(&self.buf[0..self.len]) }
    }

    pub fn as_slice_mut(&mut self) -> &mut [T] {
        // SAFETY: indexes from 0 to len are garanteed to be valid.
        unsafe { MaybeUninit::slice_assume_init_mut(&mut self.buf[0..self.len]) }
    }

    pub fn take(&mut self) -> Self {
        std::mem::take(self)
    }

    pub fn split_array_at(&mut self, index: usize) -> Self {
        let right = self.get_mut(index..).expect("index is out of bounds");

        // SAFETY: MaybeUninit is `repr(transparent)` and we are shadowing it so no UB can occur.

        let right: &mut [MaybeUninit<T>] = unsafe { std::mem::transmute(right) };

        let mut right_vec = VecArray::new();
        for i in 0..right.len() {
            unsafe {
                right_vec
                    .push(std::mem::replace(&mut right[i], MaybeUninit::zeroed()).assume_init())
                    .unwrap();
            }
        }

        right_vec
    }

    pub fn push(&mut self, element: T) -> Result<(), ArrayFullError> {
        if self.len + 1 < N {
            self.buf[self.len].write(element);
            self.len += 1;
            Ok(())
        } else {
            Err(ArrayFullError)
        }
    }

    pub fn slot_push(&mut self) -> Option<FreeSlot<'_, Self>> {
        if self.len + 1 < N {
            Some(FreeSlot { ref_mut: self })
        } else {
            None
        }
    }

    pub fn get_slot(&mut self, index: usize) -> Option<OccupiedSlot<'_, Self>> {
        if index < self.len {
            Some(OccupiedSlot {
                ref_mut: self,
                index,
            })
        } else {
            None
        }
    }

    pub fn insert(&mut self, element: T, index: usize) -> Result<(), ArrayInsertError> {
        if index > self.len {
            return Err(ArrayInsertError::Index(ArrayIndexOutOfBoudsError));
        }

        if index == self.len {
            self.push(element).map_err(ArrayInsertError::Full)?;
        } else {
            let len = self.len;
            // SAFETY: Infalliable, we shift every element after index to the right and have
            // asserted that there is enough space for it. Then insert the new element.
            unsafe {
                let p = self.buf.as_mut_ptr().add(index);
                std::ptr::copy(p, p.add(1), len - index);
                p.write(MaybeUninit::new(element));
            }
            self.len += 1;
        }

        Ok(())
    }

    pub fn push_evict_max_by_key<F, Key>(&mut self, element: T, mut get_key: F) -> Option<T>
    where
        F: FnMut(&T) -> Key,
        Key: Ord,
    {
        if self.len + 1 < N {
            self.buf[self.len].write(element);
            self.len += 1;
            None
        } else {
            let max = self.iter_mut().max_by_key(|el| get_key(el))?;
            Some(std::mem::replace(max, element))
        }
    }

    pub fn pop(&mut self) -> Option<T> {
        if self.len > 0 {
            self.len -= 1;
            let out = std::mem::replace(&mut self.buf[self.len], MaybeUninit::zeroed());
            // SAFETY: We knew that self.buf[..self.len] is valid, so the last element must be valid!
            unsafe { Some(out.assume_init()) }
        } else {
            None
        }
    }
}

impl<T: Ord, const N: usize> VecArray<T, N> {
    // If the VecArray isn't full, simply push the element on the array. If `N == 0`,
    // then it will do nothing and return None. If `N > 0` and the VecArray is full,
    // it replaces the maximum element of the vector with `element` and returns the
    // old maximum element. Note that the operation is O(N) since it may have to
    // search the entire VecArray in otder to find the maximum element.
    pub fn push_evict_max(&mut self, element: T) -> Option<T> {
        if self.len + 1 < N {
            self.buf[self.len].write(element);
            self.len += 1;
            None
        } else {
            let max = self.iter_mut().max()?;
            Some(std::mem::replace(max, element))
        }
    }
}

impl<T, const N: usize> Deref for VecArray<T, N> {
    type Target = [T];

    fn deref(&self) -> &[T] {
        unsafe { MaybeUninit::slice_assume_init_ref(&self.buf[0..self.len]) }
    }
}

impl<T, const N: usize> DerefMut for VecArray<T, N> {
    fn deref_mut(&mut self) -> &mut [T] {
        unsafe { MaybeUninit::slice_assume_init_mut(&mut self.buf[0..self.len]) }
    }
}

impl<T: Clone, const N: usize> Clone for VecArray<T, N> {
    fn clone(&self) -> Self {
        let mut buf = MaybeUninit::uninit_array();
        let len = self.len();
        // SAFETY: We know that in range 0..len, the values are valid.
        unsafe {
            for i in 0..len {
                buf[i].write(self.buf[i].assume_init_ref().clone());
            }
        }
        VecArray { buf, len }
    }
}

impl<T, const N: usize> Drop for VecArray<T, N> {
    fn drop(&mut self) {
        // SAFETY: Values in range 0..len are always valid.
        unsafe {
            for i in 0..self.len() {
                self.buf[i].assume_init_drop();
            }
        }
    }
}

impl<T: Debug, const N: usize> Debug for VecArray<T, N> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[")?;
        if self.len() > 0 {
            write!(f, "{:?}", self[0])?;

            for i in 1..self.len() {
                write!(f, ", {:?}", self[i])?;
            }
        }
        write!(f, "]")
    }
}

impl<A, const N: usize> FromIterator<A> for VecArray<A, N> {
    fn from_iter<T>(iter: T) -> Self
    where
        T: IntoIterator<Item = A>,
    {
        let iter = iter.into_iter().take(N);
        let mut arr = VecArray::new();
        for el in iter {
            arr.push(el).unwrap_or_else(|_| {
                unreachable!("Took only N elements which fits in the VecArray")
            });
        }
        arr
    }
}

impl<T, const N: usize> Default for VecArray<T, N> {
    fn default() -> Self {
        VecArray::new()
    }
}

pub struct FreeSlot<'a, T> {
    ref_mut: &'a mut T,
}

impl<'a, T, const N: usize> FreeSlot<'a, VecArray<T, N>> {
    pub fn insert(self, val: T) -> OccupiedSlot<'a, VecArray<T, N>> {
        let index = self.ref_mut.len;
        self.ref_mut.buf[index].write(val);
        self.ref_mut.len += 1;
        OccupiedSlot {
            ref_mut: self.ref_mut,
            index,
        }
    }
}

pub struct OccupiedSlot<'a, T> {
    ref_mut: &'a mut T,
    index: usize,
}

impl<'a, T, const N: usize> OccupiedSlot<'a, VecArray<T, N>> {
    pub fn replace(&mut self, val: T) -> T {
        let old = std::mem::replace(&mut self.ref_mut.buf[self.index], MaybeUninit::new(val));
        // SAFETY: We have constructed the OccupiedSlot with `index`, so it must be valid.
        unsafe { old.assume_init() }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(transparent)]
pub struct Sorted<S>(S);

impl<S, T> Deref for Sorted<S>
where
    S: DerefMut<Target = [T]>,
    T: Ord,
{
    type Target = S;

    #[inline(always)]
    fn deref(&self) -> &S {
        &self.0
    }
}

impl<S, T> DerefMut for Sorted<S>
where
    S: DerefMut<Target = [T]>,
    T: Ord,
{
    #[inline(always)]
    fn deref_mut(&mut self) -> &mut S {
        &mut self.0
    }
}

impl<T: Ord, const N: usize> Sorted<VecArray<T, N>> {
    pub fn new(mut array: VecArray<T, N>) -> Self {
        array.sort();
        Sorted(array)
    }

    pub fn push(&mut self, element: T) -> Result<(), ArrayInsertError> {
        let index = self.binary_search(&element).map_or_else(id, id);

        self.insert(element, index)
    }

    pub fn push_evict_max(&mut self, element: T) -> Option<T> {
        if N == 0 {
            panic!("Cannot push evict on 0 sized VecArray");
        }

        let index = self.binary_search(&element).map_or_else(id, id);

        let out = if self.len + 1 < N {
            None
        } else if index == self.len {
            // We return straight away because we know the VecArray is full but we would insert some
            // element that would be at the end, so we evict it immediately.
            return Some(element);
        } else {
            self.pop()
        };

        // SAFETY: We know that insert fails if the index is invalid, which it is't because we use
        // binary search on the valid slice, or if the array is full, which it can't be since we
        // we have asserted that if it was full, we popped a value. We also panic if `N == 0`.
        unsafe { self.insert(element, index).unwrap_unchecked() }
        out
    }
}

impl<T, const N: usize> Sorted<VecArray<T, N>> {
    pub fn new_by_key<F, Key>(mut array: VecArray<T, N>, get_key: F) -> Self
    where
        F: FnMut(&T) -> Key,
        Key: Ord,
    {
        array.sort_by_key(get_key);
        Sorted(array)
    }

    pub fn push_by_key<F, Key>(
        &mut self,
        element: T,
        mut get_key: F,
    ) -> Result<(), ArrayInsertError>
    where
        F: FnMut(&T) -> Key,
        Key: Ord,
    {
        let key = get_key(&element);
        let index = self
            .0
            .binary_search_by_key(&key, get_key)
            .map_or_else(id, id);

        self.0.insert(element, index)
    }

    pub fn push_evict_max_by_key<F, Key>(&mut self, element: T, mut get_key: F) -> Option<T>
    where
        F: FnMut(&T) -> Key,
        Key: Ord,
    {
        if N == 0 {
            panic!("Cannot push evict on 0 sized VecArray");
        }

        let key = get_key(&element);
        let index = self
            .0
            .binary_search_by_key(&key, get_key)
            .map_or_else(id, id);

        let out = if self.0.len + 1 < N {
            None
        } else if index == self.0.len {
            // We return straight away because we know the VecArray is full but we would insert some
            // element that would be at the end, so we evict it immediately.
            return Some(element);
        } else {
            self.0.pop()
        };

        // SAFETY: We know that insert fails if the index is invalid, which it is't because we use
        // binary search on the valid slice, or if the array is full, which it can't be since we
        // we have asserted that if it was full, we popped a value. We also panic if `N == 0`.
        unsafe {
            self.0.insert(element, index).unwrap_unchecked();
        }
        out
    }
}
