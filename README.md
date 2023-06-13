# ROS2_PathPlanningAlgorithm

Path Planning Algoritması Raporu

Giriş:
Path planning (yol planlama) algoritmaları, bir aracın belirli bir hedefe ulaşmak için uygun bir yolun hesaplanmasında kullanılır. Bu raporda, verilen bir ızgara (grid) üzerinde path planning algoritmasının nasıl çalıştığı ve kullanılan veri yapıları açıklanmaktadır. Ayrıca, aracın konumu ve çevre bilgilerini almak için kullanılan mesajların da kısa bir tanımı verilmiştir.

Izgara (Grid) Oluşturma:
Path planning algoritması için ilk adım, belirli bir alanda bir ızgara oluşturmaktır. Izgara, 0 ve 1 değerlerinden oluşan bir matris olarak temsil edilir. 0, engel olmadığını, 1 ise engel olduğunu ifade eder. Izgara, satır (rows) ve sütun (cols) bilgileri kullanılarak oluşturulur.

std::vector<std::vector<Node*>> grid(ROWS, std::vector<Node*>(COLS));

for (int i = 0; i < ROWS; i++) {
    for (int j = 0; j < COLS; j++) {
        grid[i][j] = new Node(i, j);
    }
}

Engellerin Eklenmesi:

Izgaradaki engeller, engelin x ve y konumları (ox ve oy) kullanılarak belirlenir. Engeller, ızgaradaki ilgili konumlarına girilerek işaretlenir.

std::vector<int> ox;
std::vector<int> oy;
// Engellerin x ve y konumları belirlenir

// Engeller ızgara üzerinde işaretlenir
grid[ox[i]][oy[i]]->obstacle = 1;

Başlangıç ve Hedef Noktalarının Belirlenmesi:
Path planning algoritması, başlangıç (startNode) ve hedef (endNode) noktalarını belirlemek için kullanılır. Bu noktalar, ızgaradaki ilgili konumlara göre atanır.

Node* startNode = grid[xs][ys];
Node* endNode = grid[xe][ye];





Yolun Hesaplanması:

Son adımda, A* (A-Star) gibi bir yol bulma algoritması kullanılarak başlangıç ve hedef noktaları arasındaki en kısa yol hesaplanır. Algoritma, ızgara ve başlangıç-hedef noktalarını kullanarak bir yolun vektörünü (path) döndürür.


std::vector<Node*> path = AStar(startNode, endNode, grid);

Kullanılan Mesajlar:
Path planning algoritmasının çalışması için aracın konumunu ve çevre bilgilerini alması gerekmektedir. Bu bilgileri almak için aşağıdaki mesajlar kullanılır:

/Botcamera/camera_info: Aracın ön kamerasının görüntü bilgilerini içeren mesaj.

/Botcamera/image_raw: Aracın ön kamerasının ham görüntüsünü içeren mesaj.

/cmd_vel: Aracın hareketini kontrol etmek için kullanılan mesaj (Twist türünde).

/joint_states: Aracın eklemlerinin durum bilgilerini içeren mesaj.

/odom: Aracın anlık pozisyon bilgilerini içeren mesaj (Odometry türünde).

/upper_camera/camera_info: Aracın üst kamerasının görüntü bilgilerini içeren mesaj.

/upper_camera/image_raw: Aracın üst kamerasının ham görüntüsünü içeren mesaj. Bu görüntü kullanılarak aracın çalıştığı ortamın haritası elde edilebilir ve aracın başlangıç konumu belirlenebilir.

Sonuç:
Bu raporda, path planning algoritmasının nasıl çalıştığına dair bir açıklama sunulmuştur. Algoritmanın temel adımları ve kullanılan veri yapıları anlatılmıştır. Ayrıca, aracın konumu ve çevre bilgilerini almak için kullanılan mesajlar da tanımlanmıştır. Bu bilgiler, path planning algoritmasının bir aracın yolunu planlamak için nasıl kullanıldığını anlamak için faydalıdır.


Command:
Path Plannimnng Tutorial
1) ros2 launch maze_bot maze_1_robot_camera.launch.py
2) ros2 run maze_bot bot_mapping_deneme 
3) ros2 run cpp_pubsub astar4    #run astar algortihm without vehicle configuration
3) ros2 run cpp_pubsub astar5    #run astar algortihm with vehicle configuration using Bresenham's line algorithm


Path Following Tutorial
1) ros2 launch maze_bot maze_1_robot_camera.launch.py
2) ros2 run maze_bot maze_solver 


