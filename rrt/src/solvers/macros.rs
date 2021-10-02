
#[macro_export]
macro_rules! impl_node {
    ($(impl $trait:ident$(<$($adt:ident = $val:ty),*>)? for $node:ident$(($field:ident))?;)+) => {
        $(impl_node! { @ impl $trait$(<$($adt = $val)*>)? for $node$(($field))? })+
    };

    (@ impl HasCoords for $node:ident) => {
        impl<const N: usize> kd_tree::HasCoords<N> for $node<N> {
            #[inline(always)]
            fn coords(&self) -> [f32; N] {
                self.point().coords()
            }

            #[inline(always)]
            fn point(&self) -> na::Point<f32, N> {
                *self.borrow()
            }
        }
    };

    (@ impl Borrow for $node:ident) => {
        impl<const N: usize> std::borrow::Borrow<nalgebra::Point<f32, N>> for $node<N> {
            fn borrow(&self) -> &nalgebra::Point<f32, N> {
                &self.0.p
            }
        }
    };

    (@ impl Deref<Target = $data:ty> for $node:ident($field:ident)) => {
        impl<const N: usize> std::ops::Deref for $node<N> {
            type Target = $data;
            fn deref(&self) -> &$data {
                &self.$field
            }
        }
    }
}
