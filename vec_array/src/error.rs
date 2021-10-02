
#[derive(Debug)]
pub struct ArrayFullError;

#[derive(Debug)]
pub struct ArrayIndexOutOfBoudsError;

#[derive(Debug)]
pub enum ArrayInsertError {
    Full(ArrayFullError),
    Index(ArrayIndexOutOfBoudsError),
}