// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 20-9-3.

#pragma once

#include <memory>
#include <vector>

#include <glog/logging.h>

namespace common {

// Note: Allow copy move and assign.
// FixedArray2D own the memory.
template <typename T> class FixedArray2D {
public:
  FixedArray2D() = default;
  FixedArray2D(int rows, int cols, T value = T()) : rows_(rows), cols_(cols) {
    data_ = std::make_unique<T[]>(rows * cols);
    std::fill(this->begin(), this->end(), value);
  }

  FixedArray2D(const FixedArray2D &rhs) : rows_(rhs.rows()), cols_(rhs.cols()) {
    data_ = std::make_unique<T[]>(rhs.rows() * rhs.cols());
    std::copy(rhs.begin(), rhs.end(), this->begin());
  }

  FixedArray2D(const FixedArray2D &&rhs)
      : rows_(rhs.rows()), cols_(rhs.cols()), data_(std::move(rhs.data_)) {}

  virtual ~FixedArray2D() = default;

  FixedArray2D &operator=(const FixedArray2D &rhs) {
    rows_ = rhs.rows_;
    cols_ = rhs.cols_;
    data_.release();
    data_ = std::make_unique<T[]>(rhs.rows() * rhs.cols());
    std::copy(rhs.begin(), rhs.end(), this->begin());
    return *this;
  }

  // NOTE: when you are use these APIs, you have to make sure that FixedArray2D
  // is not empty!
  const T *begin() const {
    CHECK(!this->empty());
    return data_.get();
  }
  const T *end() const {
    CHECK(!this->empty());
    return data_.get() + (rows_ * cols_);
  }
  T *begin() {
    CHECK(!this->empty());
    return data_.get();
  }
  T *end() {
    CHECK(!this->empty());
    return data_.get() + (rows_ * cols_);
  }

  const T &operator()(int row, int col) const {
    CHECK(row >= 0 && row < rows_ && col >= 0 && col < cols_);
    return data_.get()[row * cols_ + col];
  }
  T &operator()(int row, int col) {
    CHECK(row >= 0 && row < rows_ && col >= 0 && col < cols_);
    return data_.get()[row * cols_ + col];
  }

  int rows() const { return rows_; }
  int cols() const { return cols_; }

  bool empty() const { return rows_ * cols_ == 0; }

  static int cnt;

private:
  int rows_ = 0;
  int cols_ = 0;
  std::unique_ptr<T[]> data_;
};

} // namespace common
