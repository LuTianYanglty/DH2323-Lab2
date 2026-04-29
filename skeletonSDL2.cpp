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

// Task 5.2：降低分辨率到100×100，让每帧光线追踪的计算量缩小25倍，达到实时响应
const int SCREEN_WIDTH = 100;
const int SCREEN_HEIGHT = 100;
SDL2Aux *sdlAux;
int t;
vector<Triangle> triangles;

// Task 4.1
// focalLength（焦距）：值越大视野越窄，设成屏幕高度500，视野约53°，刚好看到整个Cornell Box
float focalLength = SCREEN_HEIGHT;
// cameraPos（相机位置）：Cornell Box在z∈[-1,1]，把相机放在z=-2正对着盒子
vec3  cameraPos(0.f, 0.f, -3.f);

// Task 5.3
// yaw：相机绕Y轴的偏航角（弧度），初始为0表示正对+z方向
float yaw = 0.f;
// R：由yaw构造的旋转矩阵，用来把光线方向从相机空间转到世界空间
// 绕Y轴旋转矩阵：
//   [ cos(yaw)  0  sin(yaw) ]
//   [    0      1     0     ]
//   [-sin(yaw)  0  cos(yaw) ]
mat3  R;

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
	// 计算帧时间（单位毫秒），用来让移动速度与帧率无关
	int t2 = SDL_GetTicks();
	float dt = float(t2 - t);
	t = t2;
	cout << "Render time: " << dt << " ms." << endl;

	const Uint8* keys = SDL_GetKeyboardState(NULL);
	float moveSpeed = 0.001f * dt; // 平移速度：每毫秒0.001单位
	float rotSpeed  = 0.001f * dt; // 旋转速度：每毫秒0.001弧度

	if (keys[SDL_SCANCODE_UP])   cameraPos.z += moveSpeed; // 前进
	if (keys[SDL_SCANCODE_DOWN]) cameraPos.z -= moveSpeed; // 后退

	// Task 5.3：左右键不再平移，改为修改yaw角
	if (keys[SDL_SCANCODE_LEFT])  yaw -= rotSpeed; // 相机向左转
	if (keys[SDL_SCANCODE_RIGHT]) yaw += rotSpeed; // 相机向右转

	// 每帧根据最新的yaw重新构造绕Y轴的旋转矩阵R
	// glm::mat3的列向量构造：mat3(col0, col1, col2)
	R = mat3(
		 cos(yaw), 0.f, -sin(yaw),  // 第一列（X轴基向量）
		 0.f,      1.f,  0.f,       // 第二列（Y轴基向量，旋转不影响Y）
		 sin(yaw), 0.f,  cos(yaw)   // 第三列（Z轴基向量）
	);
}

void Draw()
{
	sdlAux->clearPixels();

	// Task 4.2：遍历每个像素，向场景发射光线
	for (int y = 0; y < SCREEN_HEIGHT; ++y)
	{
		for (int x = 0; x < SCREEN_WIDTH; ++x)
		{
			// 方向向量 d = (x - W/2, y - H/2, focalLength)
			// x-W/2 和 y-H/2 是像素相对屏幕中心的偏移，focalLength是前向深度
			// 先构造相机空间的方向向量，再乘以R转到世界空间
			// 这样相机转动时，光线方向也跟着转
			vec3 dir = R * vec3(x - SCREEN_WIDTH  / 2.f,
			                    y - SCREEN_HEIGHT / 2.f,
			                    focalLength);

			Intersection isect;
			if (ClosestIntersection(cameraPos, dir, triangles, isect))
				sdlAux->putPixel(x, y, triangles[isect.triangleIndex].color); // 命中：画三角形颜色
			else
				sdlAux->putPixel(x, y, vec3(0.f, 0.f, 0.f)); // 未命中：画黑色
		}
	}
	sdlAux->render();
}