#!/usr/bin/env ruby
# -*- encoding: utf-8 -*-
#
# File: cv3stab.rb
#
# Copyright rejuvyesh <mail@rejuvyesh.com>, 2015
# License: GNU GPL 3 <http://www.gnu.org/copyleft/gpl.html>

require 'epitools'

# Atleast do the shorter videos first
VID_IN = Path['/cvgl/group/UAV_data/3-undistort/*'].sort_by{ |x| File.size(x) }
VID_OUT = '~/stabvideo/'

CMD0 = '/home/jkg/videostab'

def what(in_file, out_file)
  puts "--------------------------------------------------------------------------"
  puts "Stabilizing video:"
  puts "    in: #{in_file}"
  puts "   out: #{out_file}"
  puts "--------------------------------------------------------------------------"
  puts
end

def real_one(in_file, out_file)
  what(in_file, out_file)
  system(CMD0, in_file)
  system("ffmpeg -threads 8 -f image2 -i images/%08d.jpg -r 30 #{out_file}")
  system('rm -rf images/*')
end

VID_IN.each do |filename| 
  FileUtils.mkdir_p(VID_OUT)
  out_file = filename.with(:base => filename.base + '-real-stab', :ext => '.avi', :dir => VID_OUT)
  real_one(filename, out_file)
end
