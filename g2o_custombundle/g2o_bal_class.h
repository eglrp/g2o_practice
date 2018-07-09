#include <Eigen/Core>
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"

#include "ceres/autodiff.h"

#include "tools/rotation.h"
#include "common/projection.h"

//因为没有这样的顶点，所以要自己定义，不需要定义的顶点有相机位姿g2o::VertexSE3Expmap().       优化变量的维度和数据类型
class VertexCameraBAL : public g2o::BaseVertex<9,Eigen::VectorXd>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexCameraBAL() {}


    virtual bool read ( std::istream& /*is*/ )
    {
        return false;
    }

    virtual bool write ( std::ostream& /*os*/ ) const
    {
        return false;
    }

    // 重置
    virtual void setToOriginImpl() {}
    // 更新
    virtual void oplusImpl ( const double* update )
    {
        //这一段仿照谁写的,下面一个顶点也是  Eigen::VectorXd:动态向量   Dimension是一个静态成员变量,参考源码
        Eigen::VectorXd::ConstMapType v ( update, VertexCameraBAL::Dimension );
        _estimate += v;
    }

};

//这个不需要自己定义吧？？？
class VertexPointBAL : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexPointBAL() {}

    virtual bool read ( std::istream& /*is*/ )
    {
        return false;
    }

    virtual bool write ( std::ostream& /*os*/ ) const
    {
        return false;
    }
    //重置
    virtual void setToOriginImpl() {}
    //更新
    virtual void oplusImpl ( const double* update )
    {
        //这是不是统一格式啊
        //表达方法1
        //Eigen::Vector3d::ConstMapType v ( update );
        //_estimate += v;
        //表达方法2
        _estimate += Eigen::Vector3d(update);
    }
};

//观测值的维度，类型，连接顶点的类型
//观测值：像素坐标P258
class EdgeObservationBAL : public g2o::BaseBinaryEdge<2, Eigen::Vector2d,VertexCameraBAL, VertexPointBAL>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeObservationBAL() {}

    virtual bool read ( std::istream& /*is*/ )
    {
        return false;
    }

    virtual bool write ( std::ostream& /*os*/ ) const
    {
        return false;
    }
    //计算误差
    virtual void computeError() override   // The virtual function comes from the Edge base class. Must define if you use edge.
    {
        //两个顶点，要注意顶点的顺序
        const VertexCameraBAL* cam = static_cast<const VertexCameraBAL*> ( vertex ( 0 ) );
        const VertexPointBAL* point = static_cast<const VertexPointBAL*> ( vertex ( 1 ) );
        // 这个残差计算很奇怪????????
        //
        // 任务:把这个表达式给我改了!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!.
        // 能够吧下面那个operator函数给我屏蔽掉 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //.data()是啥意思呢??????
        ( *this ) ( cam->estimate().data(), point->estimate().data(), _error.data() );
        //参考:
        //
    }
    //???????????????
    //能屏蔽吗,不能

    template<typename T>
    bool operator() ( const T* camera, const T* point, T* residuals ) const
    {
        T predictions[2];
        CamProjectionWithDistortion ( camera, point, predictions );
        residuals[0] = predictions[0] - T ( measurement() ( 0 ) );
        residuals[1] = predictions[1] - T ( measurement() ( 1 ) );

        return true;
    }


    //如果自己写，能写成怎么样呢？
    //自动求导函数功能
    //能屏蔽掉吗
    //可以屏蔽
    /*
    virtual void linearizeOplus() override
    {
        // use numeric Jacobians
        // BaseBinaryEdge<2, Vector2d, VertexCameraBAL, VertexPointBAL>::linearizeOplus();
        // return;
        
        // using autodiff from ceres. Otherwise, the system will use g2o numerical diff for Jacobians

        const VertexCameraBAL* cam = static_cast<const VertexCameraBAL*> ( vertex ( 0 ) );
        const VertexPointBAL* point = static_cast<const VertexPointBAL*> ( vertex ( 1 ) );
        typedef ceres::internal::AutoDiff<EdgeObservationBAL, double, VertexCameraBAL::Dimension, VertexPointBAL::Dimension> BalAutoDiff;

        Eigen::Matrix<double, Dimension, VertexCameraBAL::Dimension, Eigen::RowMajor> dError_dCamera;
        Eigen::Matrix<double, Dimension, VertexPointBAL::Dimension, Eigen::RowMajor> dError_dPoint;
        double *parameters[] = { const_cast<double*> ( cam->estimate().data() ), const_cast<double*> ( point->estimate().data() ) };
        double *jacobians[] = { dError_dCamera.data(), dError_dPoint.data() };
        double value[Dimension];
        bool diffState = BalAutoDiff::Differentiate ( *this, parameters, Dimension, value, jacobians );

        // copy over the Jacobians (convert row-major -> column-major)
        if ( diffState )
        {
            _jacobianOplusXi = dError_dCamera;
            _jacobianOplusXj = dError_dPoint;
        }
        else
        {
            assert ( 0 && "Error while differentiating" );
            _jacobianOplusXi.setZero();
            _jacobianOplusXi.setZero();
        }
    }
    */
};
