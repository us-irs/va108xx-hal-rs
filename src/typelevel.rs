///! # Module support type-level programming
///
/// This module is more or less taken from the
/// [ATSAMD HAL](https://docs.rs/atsamd-hal/0.13.0/atsamd_hal/typelevel/index.html).
/// You can find a thorough explanation of the patterns used there.
use crate::Sealed;

/// Type-level version of the [None] variant
#[derive(Default)]
pub struct NoneT;
impl Sealed for NoneT {}

/// Marker trait for type identity
///
/// This trait is used as part of the [`AnyKind`](https://docs.rs/atsamd-hal/0.13.0/atsamd_hal/typelevel/index.html)
/// trait pattern. It represents the concept of type identity, because all implementors have
/// `<Self as Is>::Type == Self`. When used as a trait bound with a specific
/// type, it guarantees that the corresponding type parameter is exactly the
/// specific type. Stated differently, it guarantees that `T == Specific` in
/// the following example.
///
/// ```
/// where T: Is<Type = Specific>
/// ```
///
/// Moreover, the super traits guarantee that any instance of or reference to a
/// type `T` can be converted into the `Specific` type.
///
/// ```
/// fn example<T>(mut any: T)
/// where
///     T: Is<Type = Specific>,
/// {
///     let specific_mut: &mut Specific = any.as_mut();
///     let specific_ref: &Specific = any.as_ref();
///     let specific: Specific = any.into();
/// }
/// ```
///
/// [`AnyKind`]: #anykind-trait-pattern
pub trait Is
where
    Self: Sealed,
    Self: From<IsType<Self>>,
    Self: Into<IsType<Self>>,
    Self: AsRef<IsType<Self>>,
    Self: AsMut<IsType<Self>>,
{
    type Type;
}

/// Type alias for [`Is::Type`]
pub type IsType<T> = <T as Is>::Type;

impl<T> Is for T
where
    T: Sealed + AsRef<T> + AsMut<T>,
{
    type Type = T;
}
