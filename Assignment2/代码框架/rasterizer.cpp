// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include <array>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>

using namespace std;

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
	Vector3f vT(x,y,0);
	Vector3f AB,BC,CA,AT,BT,CT;
	Vector3f cAT,cBT,cCT;

	AB = _v[1] - _v[0];
	BC = _v[2] - _v[1];
	CA = _v[0] - _v[2];
	AT = vT - _v[0];
	BT = vT - _v[1];
	CT = vT - _v[2];

	cAT = AB.cross(AT);
	cBT = BC.cross(BT);
	cCT = CA.cross(CT);

	return cAT[2] * cBT[2] >= 0 && cAT[2] * cCT[2] >= 0 && cBT[2] * cCT[2] >= 0; //等于0表示点在边上
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
		if(use_ssaa){
			for (float y = 0; y < height; y++)
			{
				for (float x = 0; x < width; x++)
				{
					Vector3f color(0.,0.,0.);
					Vector3f point(x,y,0.);
					for(float j = start_point; j <1.0; j+=pixel_size_sm){
					for(float i = start_point; i < 1.0; i+=pixel_size_sm)
					{
						int index = get_index_ssaa(x,y,i,j);
						color += frame_buf_ssaa[index];
					}
					}
					set_pixel(point, color/(ssaa_size*ssaa_size));
				}
			}
		}
    }
}

float calz_interpolated(float x, float y, const Triangle &t, const array<Vector4f, 3>& v)
{
	auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    z_interpolated *= w_reciprocal;		
	return z_interpolated;
}


//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
	
	float up = floor(max({v[0][1],v[1][1],v[2][1]}));
	float down = floor(min({v[0][1],v[1][1],v[2][1]}));
	float left = floor(min({v[0][0],v[1][0],v[2][0]}));
	float right = floor(max({v[0][0],v[1][0],v[2][0]}));

	Vector3f* a = new Vector3f[3];
			a[0] = v[0].head<3>();
			a[1] = v[1].head<3>();
			a[2] = v[2].head<3>();
	if (use_ssaa)
	{ 	
		//buffer_resize_ssaa(ssaa_size);
		for (float y = down; y <= up; y++) 
		{
			for(float x = left; x <= right; x++)
			{
			//	sample_zindex_detect_ssaa(ssaa_size,x,y,t,v,a);
				for(float j = start_point; j < 1.0; j+=pixel_size_sm)
				{
					for(float i = start_point; i < 1.0; i+=pixel_size_sm)
					{
						if (insideTriangle(x+i,y+j,a)){
							auto[alpha, beta, gamma] = computeBarycentric2D(x+i, y+j, t.v);
    						float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    						float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    						z_interpolated *= w_reciprocal;
							int ssaa_index = get_index_ssaa(x,y,i,j);
							if(depth_buf_ssaa[ssaa_index] > z_interpolated){
								depth_buf_ssaa[ssaa_index] = z_interpolated;
								frame_buf_ssaa[ssaa_index] = t.getColor();
							}
						}

					}
				}
		
			}
		}
	} else {
		for (float y = down; y <= up; y++)
		{
			for(float x = left; x <= right; x++)
			{
				if (insideTriangle(x + 0.5,y + 0.5,a)){
					auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    				float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    				float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
   					z_interpolated *= w_reciprocal;
					int index = get_index(x, y);
					if(depth_buf[index] > z_interpolated){
						depth_buf[index] = z_interpolated;
						Vector3f point(x, y, 1);
						set_pixel(point,t.getColor());
					}
				}
			}
		}
	}	
	delete[] a;
}


void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
		std::fill(frame_buf_ssaa.begin(), frame_buf_ssaa.end(), Eigen::Vector3f{0, 0, 0});

    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
		std::fill(depth_buf_ssaa.begin(), depth_buf_ssaa.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
	frame_buf_ssaa.resize(w * h * ssaa_size * ssaa_size);
	depth_buf_ssaa.resize(w * h * ssaa_size * ssaa_size);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}
int rst::rasterizer::get_index_ssaa(int x, int y, float i, float j)
{
	int ssaa_height = height * ssaa_size;
	int ssaa_width = width * ssaa_size;
	i = int((i-start_point)/pixel_size_sm);
	j = int((j-start_point)/pixel_size_sm);
	return (ssaa_height-1-y*ssaa_size+j)*ssaa_width + x*ssaa_size + i;
}
// clang-format on
