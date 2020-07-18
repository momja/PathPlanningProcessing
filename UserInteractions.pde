Ray3 getMouseCast() {
  Vec3 w = cam.camLookAt.minus(cam.camLocation).normalized();
  Vec3 u = cross(w, cam.camUp).normalized();
  Vec3 v = cross(u, w).normalized();

  w.mul(-1);

  float m3dx = map(mouseX, 0, width, -cam.nearPlaneW/2, cam.nearPlaneW/2);
  float m3dy = map(mouseY, 0, height, -cam.nearPlaneH/2, cam.nearPlaneH/2);
  float m3dz = -1;

  float m3dx_world = m3dx*u.x + m3dy*v.x + m3dz*w.x + cam.camLocation.x;
  float m3dy_world = m3dx*u.y + m3dy*v.y + m3dz*w.y + cam.camLocation.y;
  float m3dz_world = m3dx*u.z + m3dy*v.z + m3dz*w.z + cam.camLocation.z;

  Vec3 m_world = new Vec3(m3dx_world, m3dy_world, m3dz_world);
  Vec3 rayDir = m_world.minus(cam.camLocation);
  rayDir.normalize();
  return new Ray3(cam.camLocation, rayDir);
}

void keyPressed() {
    if (key == ' ')
        paused = !paused;
    if (key == 'p')
        debugMode = !debugMode;
    if (key=='w')
        keys[0]=true;
    if (key=='a')
        keys[1]=true;
    if (key=='s')
        keys[2]=true;
    if (key=='d')
        keys[3]=true;
    if (keyCode==LEFT)
        keys[4]=true;
    if (keyCode==RIGHT)
        keys[5]=true;
    if (keyCode==UP)
        keys[6]=true;
    if (keyCode==DOWN)
        keys[7]=true;
    if (keyCode==ENTER) {
        // Set paths for bats
        for (int i = 0; i < paths.length; i++) {
            paths[i] = planPath(startPositions[i], goalPositions[i], circlePos, circleRad, numObstacles, nodePos, numNodes);
            if (paths[i] != null && paths[i].size() > 1) {
                player.batCallActive[i] = true;
                distanceToNextNode[i] = nodePos[paths[i].get(1)].minus(startPositions[i]).length();
                posAlongPath[i] = new Vec2(nodePos[paths[i].get(0)]);
                bats.tetherPoints[i] = new Vec3(posAlongPath[i].x, 5, posAlongPath[i].y);
            } else {
                player.batCallActive[i] = false;
            }
        }
        // Set path for joker
        joker.startMove();
    }
    if (key=='q') {
        // Drop a new bat signalling device
        goalPositions[nextSpawnIndex] = new Vec2(player.position2D);
        nextSpawnIndex++;
        if (nextSpawnIndex > 2) nextSpawnIndex = 0;
    }
}

void keyReleased() {
    if (key=='w')
        keys[0]=false;
    if (key=='a')
        keys[1]=false;
    if (key=='s')
        keys[2]=false;
    if (key=='d')
        keys[3]=false;
    if (keyCode==LEFT)
        keys[4]=false;
    if (keyCode==RIGHT)
        keys[5]=false;
    if (keyCode==UP)
        keys[6]=false;
    if (keyCode==DOWN)
        keys[7]=false;
}

void mouseClicked() {
    Ray3 mouseCast = getMouseCast();
    Vec3 groundCenter = new Vec3(0,-5,0);
    Vec3 groundNormal = new Vec3(0,1,0);
    float groundCastIntersectTime = dot(groundCenter.minus(cam.camLocation), groundNormal)/dot(mouseCast.direction, groundNormal);
    Vec3 intersectPoint = mouseCast.pointAtTime(groundCastIntersectTime);
    if (mouseButton == RIGHT) {
        // Launch projectile
        player.throwProjectile(intersectPoint.minus(player.position));
    } else if (mouseButton == LEFT) {
        // Launch probe
        // Get Nearest node
        Vec2 intersectPoint2D = new Vec2(intersectPoint.x, intersectPoint.z);
        player.launchProbe(intersectPoint2D);
    }
}