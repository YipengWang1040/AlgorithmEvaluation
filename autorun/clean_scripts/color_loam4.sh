# beware of safety
output_dir=$1
if [ -d ~/cw_color_loam_eval/logs/latest ]; then
mv ~/cw_color_loam_eval/logs/latest/* $output_dir/.
fi

if [ ! -d $output_dir/../../params.yaml ]; then
cp ~/cw_color_loam_eval/src/color_loam/config/params4.yaml $output_dir/../../params.yaml
fi