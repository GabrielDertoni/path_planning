#![allow(dead_code)]

use std::cell::RefCell;
use std::ops::Range;
use std::fmt::Debug;

use rand::prelude::*;
use nalgebra as na;


pub(crate) type PointN<const N: usize> = na::Point<f32, N>;

thread_local! {
    pub static RNG: RefCell<rand::rngs::StdRng> = RefCell::new(StdRng::seed_from_u64(42));
}

pub(crate) fn gen_random() -> f32 {
    // RNG.with(|rng| rng.borrow_mut().gen::<f32>())
    random()
}

pub(crate) fn gen_random_in_range<const N: usize>(range: Range<na::Point<f32, N>>) -> na::Point<f32, N> {
    /*
    let rand_vec = RNG
        .with(|rng| {
            rng.borrow_mut().gen::<na::SVector<f32, N>>()
        });
    */
    let rand_vec = random::<na::SVector<f32, N>>();
    let s = range.start.coords;
    let e = range.end.coords;
    na::Point::from(s + rand_vec.component_mul(&(e - s)))
}

#[derive(Debug, PartialEq, PartialOrd)]
pub struct PartialOrdUnwrap<T>(pub T);

impl<T> PartialOrdUnwrap<T> {
    pub fn from_ord(self) -> T {
        self.0
    }
}

impl<T: PartialOrd> std::cmp::Eq for PartialOrdUnwrap<T> {}

impl<T: PartialOrd> std::cmp::Ord for PartialOrdUnwrap<T> {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.partial_cmp(other).unwrap()
    }
}

pub(crate) trait ToOrd: PartialOrd {
    type OrdTy: Ord;

    fn to_ord(self) -> Self::OrdTy;
}

impl<T: PartialOrd> ToOrd for T {
    type OrdTy = PartialOrdUnwrap<T>;

    fn to_ord(self) -> PartialOrdUnwrap<T> {
        PartialOrdUnwrap(self)
    }
}
