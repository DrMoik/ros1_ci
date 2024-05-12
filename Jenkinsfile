pipeline {
    agent any

    stages {
        stage('Prepare Workspace') {
            steps {
                script {
                    dir('/home/user/catkin_ws/src') {
                        echo "Will check if we need to clone or just pull"
                        if (!fileExists('ros1_ci')) {
                            sh 'git clone https://github.com/DrMoik/ros1_ci.git'
                        } else {
                            dir('ros1_ci') {
                                sh 'git pull origin main'
                            }
                        }
                    }
                }
            }
        }

        stage('Build Docker Image') {
            steps {
                dir('/home/user/catkin_ws/src/ros1_ci') {
                    sh 'sudo docker build -t ros1_ci_image .'
                }
            }
        }

        stage('Manage Docker Container') {
            steps {
                script {
                    if (sh(script: "sudo docker ps --filter 'name=ros1_jen' --filter 'status=running' | grep -q ros1_jen", returnStatus: true) == 0) {
                        echo "Container ros1_jen is running. Stopping container..."
                        sh 'sudo docker stop ros1_jen'
                    } else {
                        echo "Container ros1_jen is not running."
                    }
                }
            }
        }

        stage('Run Docker Container') {
            steps {
                sh 'sudo docker run -d --rm --name ros1_jen --net=host -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ros1_ci_image'
                sleep(10) 
                sh 'sudo docker exec ros1_jen /bin/bash -c "source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash && rostest tortoisebot_waypoints waypoints_test.launch --reuse-master"'
            }
        }
    }
    
    post {
        always {
            echo 'Finish.'
        }
    }
}
