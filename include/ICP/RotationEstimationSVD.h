/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#pragma once

#include <pcl/registration/transformation_estimation_svd.h>

using namespace pcl;
using namespace pcl::registration;


template <typename PointSource, typename PointTarget, typename Scalar = float>
class RotationEstimationSVD
: public TransformationEstimationSVD<PointSource, PointTarget, Scalar> {
public:
  using Ptr =
      shared_ptr<RotationEstimationSVD<PointSource, PointTarget, Scalar>>;
  using ConstPtr = shared_ptr<
      const RotationEstimationSVD<PointSource, PointTarget, Scalar>>;

  using Matrix4 =
      typename RotationEstimationSVD<PointSource, PointTarget, Scalar>::Matrix4;

  /** \brief Inherits from TransformationEstimationSVD, but forces it to not use the
   * Umeyama method */
  RotationEstimationSVD()
  : TransformationEstimationSVD<PointSource, PointTarget, Scalar>(false)
  {}

protected:
  /** \brief Obtain a 4x4 rigid transformation matrix from a correlation matrix H = src
   * * tgt' \param[in] cloud_src_demean the input source cloud, demeaned, in Eigen
   * format \param[in] centroid_src the input source centroid, in Eigen format
   * \param[in] cloud_tgt_demean the input target cloud, demeaned, in Eigen format
   * \param[in] centroid_tgt the input target cloud, in Eigen format
   * \param[out] transformation_matrix the resultant 4x4 rigid transformation matrix
   */
  void
  getTransformationFromCorrelation(
      const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& cloud_src_demean,
      const Eigen::Matrix<Scalar, 4, 1>& centroid_src,
      const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& cloud_tgt_demean,
      const Eigen::Matrix<Scalar, 4, 1>& centroid_tgt,
      Matrix4& transformation_matrix) const {
            transformation_matrix.setIdentity();

            // Assemble the correlation matrix H = source * target'
            Eigen::Matrix<Scalar, 3, 3> H =
                (cloud_src_demean * cloud_tgt_demean.transpose()).topLeftCorner(3, 3);

            // Compute the Singular Value Decomposition
            Eigen::JacobiSVD<Eigen::Matrix<Scalar, 3, 3>> svd(
                H, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Matrix<Scalar, 3, 3> u = svd.matrixU();
            Eigen::Matrix<Scalar, 3, 3> v = svd.matrixV();

            // Compute R = V * U'
            if (u.determinant() * v.determinant() < 0) {
                for (int x = 0; x < 3; ++x)
                v(x, 2) *= -1;
            }

            Eigen::Matrix<Scalar, 3, 3> R = v * u.transpose();

            // rotated cloud
            /* Eigen::Matrix<Scalar, 4, 4> R4;
            R4.block(0, 0, 3, 3) = R;
            R4(0, 3) = 0;
            R4(1, 3) = 0;
            R4(2, 3) = 0;
            R4(3, 3) = 1;

            Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> src_ = R4 * cloud_src_demean;

            double sum_ss = 0.0f, sum_tt = 0.0f;
            for (unsigned corrIdx = 0; corrIdx < cloud_src_demean.cols(); ++corrIdx) {
                sum_ss += cloud_src_demean(0, corrIdx) * cloud_src_demean(0, corrIdx);
                sum_ss += cloud_src_demean(1, corrIdx) * cloud_src_demean(1, corrIdx);
                sum_ss += cloud_src_demean(2, corrIdx) * cloud_src_demean(2, corrIdx);

                sum_tt += cloud_tgt_demean(0, corrIdx) * src_(0, corrIdx);
                sum_tt += cloud_tgt_demean(1, corrIdx) * src_(1, corrIdx);
                sum_tt += cloud_tgt_demean(2, corrIdx) * src_(2, corrIdx);
            } */

            //float scale = sum_tt / sum_ss;
            float scale = 1;
            transformation_matrix.topLeftCorner(3, 3) = scale * R;
            const Eigen::Matrix<Scalar, 3, 1> Rc(scale * R * centroid_src.head(3));
            transformation_matrix.block(0, 3, 3, 1) = centroid_tgt.head(3) - Rc;
      };
};




template <typename PointSource, typename PointTarget, typename Scalar = float>
class ScaleEstimationSVD
: public TransformationEstimationSVD<PointSource, PointTarget, Scalar> {
public:
  using Ptr =
      shared_ptr<ScaleEstimationSVD<PointSource, PointTarget, Scalar>>;
  using ConstPtr = shared_ptr<
      const ScaleEstimationSVD<PointSource, PointTarget, Scalar>>;

  using Matrix4 =
      typename ScaleEstimationSVD<PointSource, PointTarget, Scalar>::Matrix4;

  /** \brief Inherits from TransformationEstimationSVD, but forces it to not use the
   * Umeyama method */
  ScaleEstimationSVD()
  : TransformationEstimationSVD<PointSource, PointTarget, Scalar>(false)
  {}

protected:
  /** \brief Obtain a 4x4 rigid transformation matrix from a correlation matrix H = src
   * * tgt' \param[in] cloud_src_demean the input source cloud, demeaned, in Eigen
   * format \param[in] centroid_src the input source centroid, in Eigen format
   * \param[in] cloud_tgt_demean the input target cloud, demeaned, in Eigen format
   * \param[in] centroid_tgt the input target cloud, in Eigen format
   * \param[out] transformation_matrix the resultant 4x4 rigid transformation matrix
   */
  void
  getTransformationFromCorrelation(
      const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& cloud_src_demean,
      const Eigen::Matrix<Scalar, 4, 1>& centroid_src,
      const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& cloud_tgt_demean,
      const Eigen::Matrix<Scalar, 4, 1>& centroid_tgt,
      Matrix4& transformation_matrix) const {
            transformation_matrix.setIdentity();

            // Assemble the correlation matrix H = source * target'
            Eigen::Matrix<Scalar, 3, 3> H =
                (cloud_src_demean * cloud_tgt_demean.transpose()).topLeftCorner(3, 3);

            // Compute the Singular Value Decomposition
            Eigen::JacobiSVD<Eigen::Matrix<Scalar, 3, 3>> svd(
                H, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Matrix<Scalar, 3, 3> u = svd.matrixU();
            Eigen::Matrix<Scalar, 3, 3> v = svd.matrixV();

            // Compute R = V * U'
            if (u.determinant() * v.determinant() < 0) {
                for (int x = 0; x < 3; ++x)
                v(x, 2) *= -1;
            }

            Eigen::Matrix<Scalar, 3, 3> R = v * u.transpose();

            //R = Eigen::Matrix<Scalar, 3, 3>::Identity();

            // rotated cloud
            Eigen::Matrix<Scalar, 4, 4> R4;
            R4.block(0, 0, 3, 3) = R;
            R4(0, 3) = 0;
            R4(1, 3) = 0;
            R4(2, 3) = 0;
            R4(3, 3) = 1;

            Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> src_ = R4 * cloud_src_demean;

            double sum_ss = 0.0f, sum_tt = 0.0f;
            for (unsigned corrIdx = 0; corrIdx < cloud_src_demean.cols(); ++corrIdx) {
                sum_ss += cloud_src_demean(0, corrIdx) * cloud_src_demean(0, corrIdx);
                sum_ss += cloud_src_demean(1, corrIdx) * cloud_src_demean(1, corrIdx);
                sum_ss += cloud_src_demean(2, corrIdx) * cloud_src_demean(2, corrIdx);

                sum_tt += cloud_tgt_demean(0, corrIdx) * src_(0, corrIdx);
                sum_tt += cloud_tgt_demean(1, corrIdx) * src_(1, corrIdx);
                sum_tt += cloud_tgt_demean(2, corrIdx) * src_(2, corrIdx);
            }

            float scale = sum_tt / sum_ss;
            transformation_matrix.topLeftCorner(3, 3) = scale * R;
            const Eigen::Matrix<Scalar, 3, 1> Rc(scale * R * centroid_src.head(3));
            transformation_matrix.block(0, 3, 3, 1) = centroid_tgt.head(3) - Rc;
      };
};



