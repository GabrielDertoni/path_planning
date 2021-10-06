use std::borrow::Cow;

use nalgebra as na;

use crate::utils::*;

// NOTE: currently the value used for scalar values is an f32, but this could be made
//       generic in the future.

pub struct Ray<const N: usize> {
    origin: na::Point<f32, N>,
    dir: na::SVector<f32, N>,
}

pub struct HitRecord<const N: usize> {
    first_hit: na::Point<f32, N>,
    // This value should be such that ray.origin + param * ray.dir = first_hit.
    param: f32,
    hit_count: u32,
}

pub trait Obstacle<const N: usize> {
    fn is_inside(&self, p: &na::Point<f32, N>) -> bool;
}

pub trait Bounded<const N: usize> {
    fn bounding_box(&self) -> BBox<N>;
}

pub trait SampleValid<const N: usize>: Bounded<N> {
    fn sample_valid(&self) -> Option<na::Point<f32, N>>;
}

pub trait RayCollider<const N: usize> {
    fn ray_cast(&self, ray: Ray<N>) -> Option<HitRecord<N>>;
}

pub trait DistanceToPoint<const N: usize> {
    fn distance_to_point(&self, point: na::Point<f32, N>) -> f32;
}

pub trait Convex<const N: usize> {}
pub trait FaceNormal<const N: usize> {
    fn face_normals(&self) -> Cow<'_, [na::SVector<f32, N>]>;
}

#[repr(transparent)]
pub struct RayMarching<T>(pub T);

impl<T, C, const N: usize> RayCollider<N> for RayMarching<C>
where
    C: Iterator<Item = T> + Clone,
    T: DistanceToPoint<N>,
{
    fn ray_cast(&self, ray: Ray<N>) -> Option<HitRecord<N>> {
        let mut param = 0.0;
        let mut curr = ray.origin;
        let mut min_dist = f32::INFINITY;

        while min_dist > 1e-3 && (0.0..1e4).contains(&param) {
            min_dist = self.0
                .clone()
                .map(|el| el.distance_to_point(curr).to_ord())
                .min()?.0;
            
            curr += ray.dir * min_dist;
            param += min_dist;
        }

        if min_dist <= 1e-3 {
            Some(HitRecord {
                first_hit: curr,
                param,
                hit_count: 1,
            })
        } else {
            None
        }
    }
}

#[macro_export]
macro_rules! impl_ray_marching {
    (impl<$($generics:tt),+> $ty:ty) => {
        impl<$($generics),+> for $ty<$($generics),+> {
            fn ray_cast(&self, ray: Ray<N>) -> HitRecord<N> {
                RayMarching(self).ray_cast(ray)
            }
        }
    };
}


impl<T, const N: usize> Obstacle<N> for &T
where
    T: Obstacle<N>,
{
    #[inline(always)]
    fn is_inside(&self, p: &na::Point<f32, N>) -> bool {
        (**self).is_inside(p)
    }
}

impl<const N: usize> Obstacle<N> for &dyn Obstacle<N> {
    #[inline(always)]
    fn is_inside(&self, p: &na::Point<f32, N>) -> bool {
        (**self).is_inside(p)
    }
}

impl<const N: usize> Obstacle<N> for Box<dyn Obstacle<N>> {
    #[inline(always)]
    fn is_inside(&self, p: &na::Point<f32, N>) -> bool {
        (**self).is_inside(p)
    }
}

impl<'a, O: Obstacle<N>, const N: usize> From<&'a O> for &'a dyn Obstacle<N> {
    #[inline(always)]
    fn from(obstacle: &O) -> &dyn Obstacle<N> {
        &*obstacle
    }
}

#[derive(Clone)]
pub struct BBox<const N: usize> {
    pub min_corner: na::Point<f32, N>,
    pub max_corner: na::Point<f32, N>,
}

impl<const N: usize> BBox<N> {
    pub fn new(min_corner: na::Point<f32, N>, max_corner: na::Point<f32, N>) -> BBox<N> {
        BBox {
            min_corner,
            max_corner,
        }
    }

    pub fn volume(&self) -> f32 {
        self.min_corner
            .iter()
            .zip(self.max_corner.iter())
            .map(|(min, max)| max - min)
            .product()
    }
}

impl<const N: usize> Bounded<N> for BBox<N> {
    fn bounding_box(&self) -> BBox<N> {
        self.clone()
    }
}

impl<const N: usize> SampleValid<N> for BBox<N> {
    fn sample_valid(&self) -> Option<na::Point<f32, N>> {
        None
    }
}

pub struct Polytope<const N: usize> {
    pub vertices: Vec<na::Point<f32, N>>,
}

impl<const N: usize> Polytope<N> {
    pub fn new(vertices: Vec<na::Point<f32, N>>) -> Self {
        Self { vertices }
    }
}

pub struct ConvexPolytope<const N: usize> {
    pub veritces: Vec<na::Point<f32, N>>,
}

impl<const N: usize> ConvexPolytope<N> {
    pub fn new(veritces: Vec<na::Point<f32, N>>) -> Self {
        Self { veritces }
    }
}

pub struct Sphere<const N: usize> {
    pub center: na::Point<f32, N>,
    pub radius: f32,
}

impl<const N: usize> Sphere<N> {
    pub fn new(center: na::Point<f32, N>, radius: f32) -> Self {
        Self { center, radius }
    }
}

impl<const N: usize> Obstacle<N> for Sphere<N> {
    fn is_inside(&self, p: &na::Point<f32, N>) -> bool {
        na::distance_squared(&self.center, p) <= self.radius
    }
}

#[derive(Clone, Copy)]
pub struct Rectangle<const N: usize> {
    pub corner: na::Point<f32, N>,
    pub size: na::SVector<f32, N>,
}

impl<const N: usize> Rectangle<N> {
    pub fn new(corner: na::Point<f32, N>, size: na::SVector<f32, N>) -> Self {
        Self { corner, size }
    }
}

impl<const N: usize> Obstacle<N> for Rectangle<N> {
    fn is_inside(&self, p: &na::Point<f32, N>) -> bool {
        itertools::izip!(p.iter(), self.corner.iter(), self.size.iter()).all(
            |(&p_coord, &corner_coord, &sz)| (corner_coord..corner_coord + sz).contains(&p_coord),
        )
    }
}
