# Docker and Jenkins Setup Guide

This guide covers the installation of Docker and Docker Compose, setting up xhost for display sharing, and starting the Docker service and Jenkins on Ubuntu.

## Prerequisites

- Ubuntu OS (Instructions are specifically tailored for Ubuntu)
- sudo privileges

## Installing Docker

Run the following commands in your terminal to install Docker and Docker Compose:

```bash
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
```
## Setting up xhost for Display Sharing

Run the following commands to install xhost and enable local connections to the X server:
```bash
sudo apt-get update && \
sudo apt-get install x11-xserver-utils && \
xhost +local:root
```

## Starting Services

1. **Start the Docker service:**
  ```bash
    sudo service docker start
 ```
2. **Start Jenkins:**
    Navigate to your Jenkins workspace and run the start script:
  ```bash
    cd ~/webpage_ws
    bash start_jenkins.sh
```


