#Clover Rescue Team's official Cloverside software installer


echo "All is ready to install CloverRescue. Required free disk space - 12.5 MB. Continue? y/n: "
read install
if [ $install == "y" ]
then
    wget https://bin.equinox.io/c/4VmDzA7iaHb/ngrok-stable-linux-arm.tgz -P ./ && tar xvzf ./ngrok-stable-linux-arm.tgz -C ./
    rm ./ngrok-stable-linux-arm.tgz
    git clone https://github.com/DevMBS/CRTClover.git
    echo "Please enter your data correctly, otherwise app will not work and you will need to reinstall it!"
    echo "Enter your login: "
    read login
    echo "Enter your password: "
    read password
    echo "Enter your Ngrok authtoken: "
    read authtoken
    ./ngrok authtoken $authtoken
    echo "$login;$password" >> ./CRTClover/server/login.txt
    echo "Installing succesfully completed!"
elif [ $install == "n" ]
then
    exit 0
else
    echo "Invalid value. Please re-run the installer."
    exit 0
fi
