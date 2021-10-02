impl<'a, P, const N: usize> ptree::TreeItem for &'a KDTree<P, N>
where
    P: HasCoords<N> + Clone,
{
    type Child = NodeDepthPair<'a, P, N>;

    fn write_self<W: std::io::Write>(&self, f: &mut W, _style: &ptree::Style) -> std::io::Result<()> {
        if let None = self.0 {
            write!(f, "No root")
        } else {
            write!(f, "Root")
        }
    }

    fn children<'b>(&'b self) -> std::borrow::Cow<[Self::Child]> {
        use std::borrow::Cow;

        if let Some(root) = self.0.as_ref() {
            Cow::Owned(vec![NodeDepthPair(Some((root, 0)))])
        } else {
            // No allocation is necessary for an empty Vec.
            Cow::Owned(Vec::new())
        }
    }
}

#[derive(Clone)]
pub struct NodeDepthPair<'a, P, const N: usize>(Option<(&'a Node<P, N>, usize)>);

impl<'a, P, const N: usize> ptree::TreeItem for NodeDepthPair<'a, P, N>
where
    P: HasCoords<N> + Clone,
{
    type Child = NodeDepthPair<'a, P, N>;

    fn write_self<W: std::io::Write>(&self, f: &mut W, style: &ptree::Style) -> std::io::Result<()> {
        let NodeDepthPair(opt) = self;
        if let Some((node, depth)) = opt {
            let axis = depth % N;
            let to_paint = format!("{} at axis {}", node.median.point(), axis);
            write!(f, "{}", style.paint(to_paint))
        } else {
            write!(f, "Empty branch")
        }
    }

    fn children(&self) -> std::borrow::Cow<[Self::Child]> {
        use std::borrow::Cow;

        let NodeDepthPair(opt) = self;
        if let Some((node, depth)) = opt {

            let mut children = Vec::new();

            children.push(NodeDepthPair(
                if let Some(left) = node.left.as_deref() {
                    Some((left, depth + 1))
                } else {
                    None
                }
            ));

            children.push(NodeDepthPair(
                if let Some(right) = node.right.as_deref() {
                    Some((right, depth + 1))
                } else {
                    None
                }
            ));

            Cow::Owned(children)
        } else {
            Cow::Owned(Vec::new())
        }
    }
}
