//DH2323 skeleton code, Lab2 (SDL2 version)
#include <iostream>
#include <algorithm>
#include <limits>
#include <glm/glm.hpp>
#include "SDL2Auxiliary.h"
#include "TestModel.h"

using namespace std;
using glm::vec3;
using glm::mat3;

// ----------------------------------------------------------------------------
// GLOBAL VARIABLES

const int SCREEN_WIDTH = 500;
const int SCREEN_HEIGHT = 500;
SDL2Aux *sdlAux;
int t;
vector<Triangle> triangles;

// ----------------------------------------------------------------------------
// DATA STRUCTURES

struct Intersection
{
    vec3  position;
    float distance;
    int   triangleIndex;
};


// ----------------------------------------------------------------------------
// FUNCTIONS

void Update(void);
void Draw(void);
bool ClosestIntersection(vec3 start, vec3 dir,
                         const vector<Triangle>& triangles,
                         Intersection& closestIntersection);

// ----------------------------------------------------------------------------
// RAY–TRIANGLE INTERSECTION (Möller–Trumbore)

bool ClosestIntersection(vec3 start, vec3 dir,
                         const vector<Triangle>& triangles,
                         Intersection& closestIntersection)
{
    float m = std::numeric_limits<float>::max();
    bool  found = false;

    for (int i = 0; i < (int)triangles.size(); ++i)
    {
        const Triangle& tri = triangles[i];

        // Edge vectors from v0
        vec3 e1 = tri.v1 - tri.v0;
        vec3 e2 = tri.v2 - tri.v0;
        vec3 b  = start  - tri.v0;

        // Solve [-dir, e1, e2] * [t, u, v]^T = b  via Cramer's rule
        mat3 A(-dir, e1, e2);          // columns: -dir, e1, e2
        float detA = glm::determinant(A);
        if (std::abs(detA) < 1e-8f) continue; // ray parallel to triangle

        float t_val = glm::determinant(mat3(b,   e1, e2)) / detA;
        float u     = glm::determinant(mat3(-dir, b,  e2)) / detA;
        float v     = glm::determinant(mat3(-dir, e1, b )) / detA;

        // Inequalities 4,5,6,8: t>=0, u>=0, v>=0, u+v<=1
        if (t_val >= 0.0f && u >= 0.0f && v >= 0.0f && (u + v) <= 1.0f)
        {
            if (t_val < m)
            {
                m = t_val;
                closestIntersection.position      = start + t_val * dir;  // 3D world position Task 3.2
                closestIntersection.distance      = t_val;// scalar distance 3.2
                closestIntersection.triangleIndex = i;// which triangle 
				//
                found = true;
            }
        }
    }
    return found;
}

int main( int argc, char* argv[] )
{
	sdlAux = new SDL2Aux(SCREEN_WIDTH, SCREEN_HEIGHT);//在电脑屏幕上弹出一个窗口
	t = SDL_GetTicks();	// Set start value for timer.按下秒表，记录现在的系统时间。
	LoadTestModel(triangles);//准备了一些三角形，虽然在这个代码片段里还没画出来，但属于前期准备

	while (!sdlAux->quitEvent())
	{
		Update();
		Draw();
	}
	sdlAux->saveBMP("screenshot.bmp");
	return 0;//画面刷新和保存图片
}

void Update(void)
{
	// Compute frame time:ß  计算“画一幅画需要多长时间”
	int t2 = SDL_GetTicks();
	float dt = float(t2-t);
	t = t2;
	cout << "Render time: " << dt << " ms." << endl;
}

void Draw()
{
	// Draw the scene:
	sdlAux->clearPixels();

	for( int y=0; y<SCREEN_HEIGHT; ++y )
	{
		for( int x=0; x<SCREEN_WIDTH; ++x )
		{
			vec3 color( 1, 0.5, 0.5 );
			sdlAux->putPixel(x, y, color);
		}
	}
	sdlAux->render();
}