Setup docker

Run 

sudo apt-get update && \
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y apt-transport-https ca-certificates curl software-properties-common && \
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add - && \
sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" && \
sudo apt-get update && \
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y docker-ce=5:20.10.12~3-0~ubuntu-focal docker-ce-cli=5:20.10.12~3-0~ubuntu-focal containerd.io && \
sudo curl -L "https://github.com/docker/compose/releases/download/1.25.0/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose && \
sudo chmod +x /usr/local/bin/docker-compose && \
sudo usermod -aG docker $USER && \
newgrp docker


sudo apt-get update && \
sudo apt-get install x11-xserver-utils && \
xhost +local:root

sudo service docker start


cd ~/catkin_ws/src/ros1_ci && docker build -t ros1_ci_image .

docker run -it --net=host -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ros1_ci_image