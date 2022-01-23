#Clover Rescue Team's official Cloverside software installer

echo "RescueClover installation started."
git clone https://github.com/DevMBS/CRTClover.git
npm cache clean -f && npm install -g n && npx n install stable
cd ./CRTClover && npm install --save child_process roslib fs socket.io-client
mv login.txt ./CRTClover
echo "Installation successfully completed!"