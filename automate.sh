if [ $# -lt 1 ];then
	dir=.
else
	dir=$1
fi

for i in $(ls -d $dir/*)
do
	python rosbag2txt.py ${i%%/}
done
