#!/bin/sh

rm -rf bar
mkdir bar
cd foo
num=5001;
for file in *; do
    # if [ $(( $num % 2 )) -eq 0 ]; then
    cp $file "$(printf '../bar/%04d.png' $num)";
    # fi
    ((num++));
done

cd ../pngs
num=5000;
for file in *; do
    if [ $(( $num % 4 )) -eq 0 ]; then # https://bash.cyberciti.biz/guide/Continue_statement
        # http://stackoverflow.com/questions/7450818/rename-all-files-in-directory-from-filename-h-to-filename-half
        # http://stackoverflow.com/questions/3211595/renaming-files-in-a-folder-to-sequential-numbers
        cp $file "$(printf '../bar/%04d.png' $num)";
    fi
    ((num--));
done

cd ..
ffmpeg -framerate 24 -pattern_type glob -i 'bar/*.png' -c:v libx264 -r 30 -pix_fmt yuv420p out.mp4
# mencoder bar/*.png -mf fps=24 -ovc lavc -lavcopts vcodec=msmpeg4v2 -oac copy -o output.avi
