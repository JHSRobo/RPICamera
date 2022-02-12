while true;
do
  git describe --tags --abbrev=0 | sudo nc -k -l -p 80 -q 1
done
