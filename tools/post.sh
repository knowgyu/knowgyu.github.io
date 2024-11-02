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
categories: [MLOps, Kubeflow-Pipeline-Run]
tags: [MLOps, Kubeflow]
---

여기에 글 내용을 작성하세요.
EOM
echo "New post created: $filename"
