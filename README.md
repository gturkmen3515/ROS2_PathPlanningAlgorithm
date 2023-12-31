# ROS2_PathPlanningAlgorithm

**Dependencies :**

**ROS Based :**

* sudo apt-get install ros-foxy-joint-state-publisher
* sudo apt-get install ros-foxy-robot-state-publisher
* sudo apt-get install ros-foxy-gazebo-plugins

**Python Based :**

* pip install pygame
* pip install setuptools==58.1.0
* pip install opencv-contrib-python
* pip install numpy


**Path Planning Algoritması Raporu**

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
Path Planning Tutorial
1) ros2 launch maze_bot maze_1_robot_camera.launch.py
2) ros2 run maze_bot bot_mapping_deneme 
3) ros2 run cpp_pubsub  main    #run astar algortihm with vehicle configuration using Bresenham's line algorithm 20 second

Path Following Tutorial
1) ros2 launch maze_bot maze_1_robot_camera.launch.py
2) ros2 run maze_bot maze_solver 

Path Planning optimized
1) ros2 launch maze_bot maze_1_robot_camera.launch.py
2) ros2 run maze_bot bot_mapping_deneme 
3) ros2 run cpp_pubsub  astar_opt    #1 second

_**path_planning_deneme**_
1) ros2 launch maze_bot maze_1_robot_camera.launch.py
2) ros2 run maze_bot bot_mapping_deneme
3) ros2 run path_planning_deneme main    #1 second

MazeSolver.h dosyasında 109 ve 110 satırlarda araçın genişligi ve boyu eklenmiştir.

`const int vehicleWidth = gridsize_.data[6];  // Define the width of the vehicle
`

`const int vehicleLength = gridsize_.data[7]; // Define the length of the vehicle
`

126-145 satırları arasında örnek configuration space algoritması uygulanmıştır.Burda Bresenham's line algorithması uygulanmıştır.Algorithmanın h file isCellOnLine.h dır.

Code Configuration space

    for (int i = 0; i < obsx_.data.size(); i++)
    {
        int obsx = std::move(obsx_.data[i]);
        int obsy = std::move(obsy_.data[i]);

        // Mark the cells along the line segment representing the vehicle's dimensions
        for (int dx = -vehicleWidth/2; dx <= vehicleWidth/2; dx++) {
            for (int dy = -vehicleLength/2; dy <= vehicleLength/2; dy++) {
                int x = obsx + dx;
                int y = obsy + dy;

                // Check if the cell lies on the line segment
                if (isCellOnLine(obsx, obsy, x, y)) {
                    if (x >= 0 && x < ROWS && y >= 0 && y < COLS) {
                        grid[x][y]->obstacle = 1;
                    }
                }
            }
        }
    }

 isCellOnLine.h kodu, bir hücrenin bir çizgi parçası üzerinde olup olmadığını kontrol eden isCellOnLine adlı bir işlevi tanımlar. Fonksiyon dört parametre alır: x1 ve y1, doğru parçasının başlangıç noktasının koordinatlarıdır aynı zamanda engel kordinatlarıdır ve x2 ve y2, doğru parçasının bitiş noktasının koordinatlarıdır buda engel noktalarının araç boyutu kadar ilerletilmesidir.

İşlev, hücrenin doğru parçası üzerinde olup olmadığını belirlemek için Bresenham'ın çizgi algoritmasını kullanır. Algoritma, hat boyunca başlangıç noktasından bitiş noktasına yinelemeli olarak hareket ederek çalışır ve her hücrenin hattın bir parçası olup olmadığını kontrol eder. Algoritma, çizginin yönüne (sx ve sy değişkenleri) bağlı olarak x ve y koordinatlarını artırır veya azaltır ve bir sonraki hücreyi belirlemek için hata değerini (err) ayarlar.

Kontrol edilen geçerli hücre, doğru parçasının bitiş noktasıyla aynıysa, işlev, hücrenin doğru parçası üzerinde olduğunu belirten doğru değerini döndürür. İşlev, bir eşleşme bulmadan döngüyü tamamlarsa, dolaylı olarak false döndürür.
