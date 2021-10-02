#![allow(dead_code)]

use std::cell::RefCell;
use std::mem::MaybeUninit;
use std::ops::{ Deref, DerefMut, Drop, Range };
use std::convert::identity as id;
use std::fmt::{ self, Debug };

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
pub struct OrdF32(pub f32);

impl std::cmp::Eq for OrdF32 {}

impl std::cmp::Ord for OrdF32 {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.partial_cmp(other).unwrap()
    }
}