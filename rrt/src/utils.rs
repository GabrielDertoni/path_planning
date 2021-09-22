use std::ops::Range;

use rand::prelude::*;
use nalgebra as na;

pub(crate) fn gen_random_in_range<const N: usize>(range: Range<na::Point<f32, N>>) -> na::Point<f32, N> {
    let rand_vec = random::<na::SVector<f32, N>>();
    let s = range.start.coords;
    let e = range.end.coords;
    na::Point::from(s + rand_vec.component_mul(&(e - s)))
}
