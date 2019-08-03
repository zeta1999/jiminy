# Robotpkg dependencies
sudo sh -c "echo 'deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub bionic robotpkg' >> /etc/apt/sources.list.d/robotpkg.list" && \
curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add -
sudo apt update

# Python 2.7 dependencies installation procedure
sudo apt install -y robotpkg-py27-pinocchio robotpkg-py27-qt4-gepetto-viewer-corba

# Python 3.6 dependencies installation procedure
sudo apt install -y robotpkg-py36-pinocchio robotpkg-py36-qt4-gepetto-viewer-corba

# Tensorflow 1.13 dependencies (Cuda 10.1 and CuDNN 7.6)
Amazing tutorial: https://medium.com/better-programming/install-tensorflow-1-13-on-ubuntu-18-04-with-gpu-support-239b36d29070

# Install open AI Gym along with some toy models
pip install gym[atari,box2d,classic_control]
