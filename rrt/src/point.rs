use std::rc::Rc;
use std::borrow::Borrow;

use nalgebra as na;

use crate::kd_tree::HasCoords;

#[derive(Debug, Clone)]
pub struct Point<const N: usize>(pub Rc<PointData<N>>);

#[derive(Debug, Clone)]
pub struct PointData<const N: usize> {
    pub p: na::Point<f32, N>,
    pub connected: Option<Point<N>>,
}

impl<const N: usize> Borrow<na::Point<f32, N>> for Point<N> {
    #[inline(always)]
    fn borrow(&self) -> &na::Point<f32, N> { &self.0.p }
}

impl<'a, const N: usize> Borrow<na::Point<f32, N>> for &'a Point<N> {
    #[inline(always)]
    fn borrow(&self) -> &na::Point<f32, N> { (*self).borrow() }
}

impl<const N: usize> Point<N> {
    pub fn new(p: na::Point<f32, N>) -> Point<N> {
        Point(Rc::new(PointData { p, connected: None }))
    }

    pub fn new_connected(p: na::Point<f32, N>, other: Point<N>) -> Point<N> {
        Point(Rc::new(PointData { p, connected: Some(other) }))
    }

    #[inline(always)]
    pub fn connected(&self) -> &Option<Point<N>> {
        &self.0.connected
    }

    pub fn ptr_eq(&self, other: &Point<N>) -> bool {
        Rc::ptr_eq(&self.0, &other.0)
    }
}

impl<const N: usize> HasCoords<N> for Point<N> {
    #[inline(always)]
    fn get_axis(&self, axis: usize) -> Option<f32> {
        if axis < N {
            Some(self.0.p[axis])
        } else {
            None
        }
    }

    #[inline(always)]
    fn point(&self) -> na::Point<f32, N> { *self.borrow() }
}