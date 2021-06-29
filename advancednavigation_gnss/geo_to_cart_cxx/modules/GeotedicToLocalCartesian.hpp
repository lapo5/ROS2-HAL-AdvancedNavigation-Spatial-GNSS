#ifndef MY_GEO_TO_CART
#define MY_GEO_TO_CART


#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include <xtensor/xtensor.hpp>
#include <xtensor-blas/xlinalg.hpp>
#include <xtensor/xadapt.hpp>
#include <xtensor/xarray.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xdynamic_view.hpp>
#include <xtensor-blas/xlinalg.hpp>

namespace py = pybind11;
using namespace GeographicLib;
using namespace std;

template<typename T>
using xarray_view = decltype(xt::adapt((T *)nullptr, 1, xt::no_ownership(), typename xt::xarray<T>::shape_type{1, 1}));

template<typename T>
xarray_view<T> xarray_view_placeholder = xt::adapt((T *)nullptr, 1, xt::no_ownership(), typename xt::xarray<T>::shape_type{1, 1});

#define MAP_BUF_TO_XARRAY_VIEW(buf, view)                               \
    view(xt::adapt((decltype(buf)::value_type *)buf.request().ptr,      \
                    buf.request().shape[0] * buf.request().shape[1],    \
                    xt::no_ownership(),                                 \
                    xt::xarray<decltype(buf)::value_type>::shape_type{  \
                        buf.request().shape[0], buf.request().shape[1]}))



class GeotedicToLocalCartesian{

public:

    // Inputs
    xarray_view<double> geotedic_pose;
    xarray_view<double> goetedic_confidences;

    // Outputs
    xarray_view<double> cartesian_pose;
    xarray_view<double> cartesian_confidences;

    std::vector<double> M;
    xt::xarray<double> T_matrix;
    xt::xarray<double> geo_confidences_M;

    bool first_time;

    double x, y, z;
    double start_lat, start_lon, start_alt;

protected:
    Geocentric* earth;

    LocalCartesian* proj;

public:
    GeotedicToLocalCartesian(py::array_t<double, py::array::c_style | py::array::forcecast> _geotedic_pose,
                                py::array_t<double, py::array::c_style | py::array::forcecast> _goetedic_confidences,
                                py::array_t<double, py::array::c_style | py::array::forcecast> _cartesian_pose,
                                py::array_t<double, py::array::c_style | py::array::forcecast> _cartesian_confidences)
    : MAP_BUF_TO_XARRAY_VIEW(_geotedic_pose, geotedic_pose)
    , MAP_BUF_TO_XARRAY_VIEW(_goetedic_confidences, goetedic_confidences)
    , MAP_BUF_TO_XARRAY_VIEW(_cartesian_pose, cartesian_pose)
    , MAP_BUF_TO_XARRAY_VIEW(_cartesian_confidences, cartesian_confidences)

    {
        earth = new Geocentric(Geocentric::WGS84());

        T_matrix = xt::eye<double>({3, 3});
        geo_confidences_M = xt::zeros<double>({3, 3});

        first_time = true;
    }

    void reset(){
	start_lat = geotedic_pose(0, 0);
	start_lon = geotedic_pose(1, 0);
	start_alt = geotedic_pose(2, 0);
        proj = new LocalCartesian(start_lat, start_lon, start_alt, *earth);
        cartesian_pose(0, 0) = 0.0;
        cartesian_pose(1, 0) = 0.0;
        cartesian_pose(2, 0) = 0.0;
        cartesian_confidences = xt::eye<double>({3, 3});

    }

    void compute(){
        if(first_time){
            this->reset();
            first_time = false;
        }
        else{
            proj->Forward(geotedic_pose(0, 0), geotedic_pose(1, 0), geotedic_pose(2, 0), x, y, z, M);

            if(M.size() == 9){
                T_matrix(0, 0) = M.at(0);
                T_matrix(0, 1) = M.at(1);
                T_matrix(0, 2) = M.at(2);
                T_matrix(1, 0) = M.at(3);
                T_matrix(1, 1) = M.at(4);
                T_matrix(1, 2) = M.at(5);
                T_matrix(2, 0) = M.at(6);
                T_matrix(2, 1) = M.at(7);
                T_matrix(2, 2) = M.at(8);
            }
            else{
                T_matrix = xt::eye<double>({3, 3});
            }
            cartesian_pose(0, 0) = x * 1000.0;
            cartesian_pose(1, 0) = y * 1000.0;
            cartesian_pose(2, 0) = z * 1000.0;
            geo_confidences_M(0, 0) = goetedic_confidences(0, 0) * 1000.0;
            geo_confidences_M(1, 1) = goetedic_confidences(1, 0) * 1000.0;
            geo_confidences_M(2, 2) = goetedic_confidences(2, 0) * 1000.0;

            cartesian_confidences = xt::linalg::dot(T_matrix, geo_confidences_M);

        }
    }

};

#endif //MY_GEO_TO_CART
