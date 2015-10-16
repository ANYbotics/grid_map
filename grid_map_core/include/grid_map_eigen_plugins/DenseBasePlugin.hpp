Scalar numberOfFinites() const
{
  if (SizeAtCompileTime==0 || (SizeAtCompileTime==Dynamic && size()==0)) return Scalar(0);
  return Scalar((derived().array() == derived().array()).count());
}

Scalar sumOfFinites() const
{
  if (SizeAtCompileTime==0 || (SizeAtCompileTime==Dynamic && size()==0)) return Scalar(0);
  return Scalar(this->redux(Eigen::internal::scalar_sum_op_of_finites<Scalar>()));
}

Scalar meanOfFinites() const
{
  return Scalar(this->redux(Eigen::internal::scalar_sum_op_of_finites<Scalar>())) / this->numberOfFinites();
}
