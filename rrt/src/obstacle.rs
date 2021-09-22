use nalgebra as na;

// NOTE: currently the value used for scalar values is an f32, but this could be made
//       generic in the future.

pub trait Obstacle<const N: usize> {
    fn is_inside(&self, p: &na::Point<f32, N>) -> bool;
}

pub struct Polytope<const N: usize> {
    pub vertices: Vec<na::Point<f32, N>>,
}

impl<const N: usize> Polytope<N> {
    pub fn new(vertices: Vec<na::Point<f32, N>>) -> Self { Self { vertices } }
}

pub struct ConvexPolytope<const N: usize> {
    pub veritces: Vec<na::Point<f32, N>>,
}

impl<const N: usize> ConvexPolytope<N> {
    pub fn new(veritces: Vec<na::Point<f32, N>>) -> Self { Self { veritces } }
}

pub struct Sphere<const N: usize> {
    pub center: na::Point<f32, N>,
    pub radius: f32,
}

impl<const N: usize> Sphere<N> {
    pub fn new(center: na::Point<f32, N>, radius: f32) -> Self { Self { center, radius } }
}

impl<const N: usize> Obstacle<N> for Sphere<N> {
    fn is_inside(&self, p: &na::Point<f32, N>) -> bool {
        na::distance_squared(&self.center, p) <= self.radius
    }
}

pub struct Rectangle<const N: usize> {
    pub corner: na::Point<f32, N>,
    pub size: na::SVector<f32, N>,
}

impl<const N: usize> Rectangle<N> {
    pub fn new(corner: na::Point<f32, N>, size: na::SVector<f32, N>) -> Self { Self { corner, size } }
}

impl<const N: usize> Obstacle<N> for Rectangle<N> {
    fn is_inside(&self, p: &na::Point<f32, N>) -> bool {
        itertools::izip!(p.iter(), self.corner.iter(), self.size.iter())
            .all(|(&p_coord, &corner_coord, &sz)|
                (corner_coord..corner_coord + sz).contains(&p_coord)
            )
    }
}