#include <unistd.h>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "pangolin/pangolin.h"

// path to trajectory file
std::string trajectory_file = "../examples/trajectory.txt";

void DrawTrajectory(std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>);

int main(int argc, char **argv)
{
    // Eigen::Transform : N차원 공간에서의 Homogeneous transform을 나타냄.
    // Eigen 라이브러리의 Isometry3d 객체를 저장하는 벡터 선언. Eigen::Isometry3d: 3차원 공간에서의 이동과 회전을 표현
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;

    // trajectory_file을 읽기 위한 입력 파일 스트림을 생성
    std::ifstream fin(trajectory_file);
    if (!fin)  // 파일 스트림이 제대로 열리지 않앗다면 메시지 출력 후 종료
    {
        std::cout << "cannot find trajectory file at " << trajectory_file << std::endl;
        return 1;
    }

    // 파일의 끝에 도달할 대 까지 루프를 돌며 각 행을 읽음
    while (!fin.eof())
    {
        double time, tx, ty, tz, qx, qy, qz, qw;
        // 각 행에서 시간(time), 위치(tx, ty, tz), 쿼터니언(qx, qy, qz, qw) 값을 읽음
        fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        // 읽어온 쿼터니언 값을 이용해 회전에 대한 Isometry3d 객체 생성
        Eigen::Isometry3d Twr(Eigen::Quaterniond(qw, qx, qy, qz));
        std::cout << Twr.matrix() << std::endl;
        std::cout << std::endl;
        // Twr 객체를 translation mat 부분을 적용, 참조를 반환
        Twr.pretranslate(Eigen::Vector3d(tx, ty, tz));
        std::cout << Twr.matrix() << std::endl;
        // 변환된 객체를 poses 벡터에 추가
        poses.push_back(Twr);
    }
    // 읽어온 포즈의 개수 출력
    std::cout << "read total " << poses.size() << " pose entries" << std::endl;

    // draw trajectory in pangolin
    DrawTrajectory(poses);
    return 0;
}

/*******************************************************************************************/
void DrawTrajectory(std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses) {
    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glLineWidth(2);
        for (size_t i = 0; i < poses.size(); i++) {
            // 画每个位姿的三个坐标轴
            Eigen::Vector3d Ow = poses[i].translation();
            Eigen::Vector3d Yw = poses[i] * (0.1 * Eigen::Vector3d(0, 1, 0));
            Eigen::Vector3d Xw = poses[i] * (0.1 * Eigen::Vector3d(1, 0, 0));
            Eigen::Vector3d Zw = poses[i] * (0.1 * Eigen::Vector3d(0, 0, 1));
            glBegin(GL_LINES);
            glColor3f(1.0, 0.0, 0.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);
            glVertex3d(Xw[0], Xw[1], Xw[2]);
            glColor3f(0.0, 1.0, 0.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);
            glVertex3d(Yw[0], Yw[1], Yw[2]);
            glColor3f(0.0, 0.0, 1.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);
            glVertex3d(Zw[0], Zw[1], Zw[2]);
            glEnd();
        }
        // 画出连线
        for (size_t i = 0; i < poses.size(); i++) {
            glColor3f(0.0, 0.0, 0.0);
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}