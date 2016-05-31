#step 1 generate eclipse project
catkin build  --force-cmake -G"Eclipse CDT4 - Unix Makefiles" 

#step 2 generate .project files
ROOT=$PWD 
cd build
for PROJECT in `find $PWD -name .project`; do
    DIR=`dirname $PROJECT`
    echo $DIR
    cd $DIR
    awk -f $(rospack find mk)/eclipse.awk .project > .project_with_env && mv .project_with_env .project
done
cd $ROOT

echo "The Next Thing to do is : open elcipse then File --> Import --> Existing projects into workspace, select the "src" path"

