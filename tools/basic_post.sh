#!/bin/bash
read -p "Post title: " title
filename="_posts/$(date +%Y-%m-%d)-${title// /-}.md"
cat >$filename <<-EOM
---
title: "$title"
author: knowgyu
description: " "
date: $(date +%Y-%m-%d) $(date +%H:%M:%S) +0900
math: true
categories: [Embedded System, ROS]
tags: [ROS, 자율주행, SLAM]
---

여기에 글 내용을 작성하세요.

EOM
echo "New post created: $filename"
