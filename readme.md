SWARM ROBOT SIMULATION
======================

Project ini merupakan simulasi Swarm Robot berbasis ROS 2, yang memanfaatkan Webots, Gazebo, dan RViz2 untuk visualisasi serta pengendalian robot.
Dengan project ini, kamu dapat mempelajari dan menjalankan simulasi pergerakan robot swarm secara interaktif.


DEMO SIMULASI
-------------
![Image](https://github.com/user-attachments/assets/7874a16f-f11b-4c1c-8061-7b7766401114)

INSTALASI & PERSIAPAN
---------------------
1. Clone repository ini atau download dalam bentuk .zip
   git clone https://github.com/username/swarm-robot-simulation.git

2. Pindahkan project ke Home Directory
   mv swarm-robot-simulation ~/swarm

3. Masuk ke workspace ROS2
   cd ~/swarm/ros2_ws


MENJALANKAN SIMULASI
--------------------
Untuk memulai simulasi, jalankan perintah berikut:

   source install/setup.bash
   ros2 launch swarm_launch swarm_simulation.launch.py

Jika berhasil, maka secara otomatis akan terbuka:
- RViz2   : Visualisasi sensor & robot
- Webots  : Simulasi lingkungan 3D
- Gazebo  : Simulasi fisika robot


PANDUAN PENGGUNAAN
------------------
1. Jalankan perintah di atas untuk memulai simulasi.
2. Amati visualisasi robot di RViz2.
3. Perhatikan pergerakan swarm di Gazebo maupun Webots.
4. Robot akan bergerak sesuai konfigurasi yang sudah disiapkan dalam launch file.


FITUR
-----
- Simulasi multi-robot swarm
- Integrasi dengan Gazebo, Webots, dan RViz2
- Mendukung pengembangan algoritma path planning & koordinasi robot
- Mudah dijalankan hanya dengan satu perintah


KONTRIBUSI
----------
Kontribusi selalu terbuka! Jika ingin menambahkan fitur baru atau memperbaiki bug:
1. Fork repository ini.
2. Buat branch baru.
3. Lakukan perubahan.
4. Ajukan Pull Request.


LISENSI
-------
Project ini dirilis di bawah lisensi MIT.
Silakan cek file LICENSE untuk detail lebih lanjut.
