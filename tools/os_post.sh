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
categories: [Embedded System, 임베디드 OS 개발 프로젝트]
tags: [Embedded System, RTOS, Firmware, OS]
---

![image.png](/assets/img/OS/OS000.jpg){: .w-25}
<br>

**본 프로젝트는 이만우 저자님의 "임베디드 OS 개발 프로젝트" 교재를 따라 RTOS를 만드는 것을 목표로 합니다.**

<br>

***

[여기에 글 내용을 작성하세요.]


## 참고
***

참고 깃허브 : [https://github.com/navilera/Navilos](https://github.com/navilera/Navilos)

이만우 저자님의 블로그 주소 : [https://kldp.org/node/162560](https://kldp.org/node/162560)

EOM
echo "New post created: $filename"
