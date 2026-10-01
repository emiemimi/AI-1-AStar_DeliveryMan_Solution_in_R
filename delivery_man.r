library("DeliveryMan")

myFunction <- function(trafficMatrix, carInfo, packageMatrix) {
  # What is our goal?
  if(carInfo$load == 0) {
      carInfo$mem$goal <- nextPickup(trafficMatrix,carInfo,packageMatrix)
    } else {
      carInfo$mem$goal <- packageMatrix[carInfo$load, c(3,4)]
  }
  # How do we get there?
  carInfo$nextMove <- nextMoveAStar(trafficMatrix,carInfo)
  print(carInfo$nextMove)
  return(carInfo)
}

# Find the nearest pickup location for an undelivered package
nextPickup <- function(trafficMatrix, carInfo, packageMatrix) {
  distanceVector = abs(packageMatrix[,1] - carInfo$x) + abs(packageMatrix[,2] - carInfo$y)
  distanceVector[packageMatrix[,5] != 0] = Inf
  return(packageMatrix[which.min(distanceVector), c(1,2)])
}

# find the best path according to aStar, and get corresponding first move
nextMoveAStar <- function(trafficMatrix, carInfo) {
  if (carInfo$mem$goal[1] == carInfo$x & carInfo$mem$goal[2] == carInfo$y) {return(5)}
  else {
    n = ncol(trafficMatrix$hroads)
    aStar <- list(
      currX = carInfo$x,
      currY = carInfo$y,
      currPath = list(),
      costToHere = 0,
      frontier = list(),
      heuristicMatrix = makeHeuristicMatrix(carInfo, n)
    )
    while (aStar$currX != carInfo$mem$goal[1] | aStar$currY != carInfo$mem$goal[2]) {
      reachableNodes = getReachableNodes(trafficMatrix, aStar, n)
      aStar <- updateFrontier(aStar, reachableNodes)
      aStar <- expandBestNode(aStar)
    }
    nextX <- aStar$currPath[[1]][1]
    nextY <- aStar$currPath[[1]][2]
    if (carInfo$x < nextX) {return(6)} # go right
    else if (carInfo$x > nextX) {return(4)} # go left
    else if (carInfo$y < nextY) {return(8)} # go up
    else {return(2)} # go down
  }
}

# build the matrix that contains, for each node, the manhattan distance to the goal
makeHeuristicMatrix <- function(carInfo, n) {
  heuristicMatrix = matrix(nrow = n, ncol = n)
  for (x in 1:n) {
    for (y in 1:n) {
      heuristicMatrix[x, y] <- (abs(carInfo$mem$goal[1] - x) + abs(carInfo$mem$goal[2] - y)) * 3
    }
  }
  return(heuristicMatrix)
}

# add / update nodes the list of nodes to be expanded
updateFrontier <- function(aStar, reachableNodes) {
  if (length(aStar$frontier) == 0) {
    for (reachableNode in reachableNodes) {
      aStar$frontier[[length(aStar$frontier) + 1]] <- reachableNode
    }
  }
  else {
    for (reachableNode in reachableNodes) {
      matchIndex <- which(sapply( # check if the node is already in the frontier
        aStar$frontier,
        function(frontierNode) {frontierNode$x == reachableNode$x & frontierNode$y == reachableNode$y}
      ))
      if (length(matchIndex) > 0) { # this length should only ever be 0 or 1
        # replace the node if the new cost is lower
        if (reachableNode$totalCost < aStar$frontier[[matchIndex[1]]]$totalCost) {
          aStar$frontier[[matchIndex[1]]] <- reachableNode
        }
      } else { # add the node to the frontier
        aStar$frontier[[length(aStar$frontier) + 1]] <- reachableNode
      }
    }
  }
  return(aStar)
}

# get the list of nodes around current position
getReachableNodes <- function(trafficMatrix, aStar, n) {
  # if current position not on a border, 2 reachable nodes per axis
  # each node: coordinates, cost (accumulated traffic), heuristic, total cost, and path
  # each node's path is a list of node coordinates, where the first element is the start of the path
  reachableNodes <- list()
  path <- aStar$currPath
  y <- aStar$currY
  if(aStar$currX == 1) { # add right node only
    x <- aStar$currX + 1
    cost <- trafficMatrix$hroads[aStar$currX, aStar$currY] + aStar$costToHere
    heuristic <- aStar$heuristicMatrix[x, y]
    path[[length(path) + 1]] = c(x, y)
    reachableNodes[[length(reachableNodes) + 1]] <- list(x = x, y = y, cost = cost, heuristic = heuristic, totalCost = cost + heuristic, path = path)
  } else if (aStar$currX == n) { # add left node only
    x <- aStar$currX - 1
    cost <- trafficMatrix$hroads[aStar$currX - 1, aStar$currY] + aStar$costToHere
    heuristic <- aStar$heuristicMatrix[x, y]
    path[[length(path) + 1]] = c(x, y)
    reachableNodes[[length(reachableNodes) + 1]] <- list(x = x, y = y, cost = cost, heuristic = heuristic, totalCost = cost + heuristic, path = path)
  } else { # add left and right nodes
    x <- aStar$currX + 1
    cost <- trafficMatrix$hroads[aStar$currX, aStar$currY] + aStar$costToHere
    heuristic <- aStar$heuristicMatrix[x, y]
    path[[length(path) + 1]] = c(x, y)
    reachableNodes[[length(reachableNodes) + 1]] <- list(x = x, y = y, cost = cost, heuristic = heuristic, totalCost = cost + heuristic, path = path)
    x <- aStar$currX - 1
    cost <- trafficMatrix$hroads[aStar$currX - 1, aStar$currY] + aStar$costToHere
    heuristic <- aStar$heuristicMatrix[x, y]
    path <- aStar$currPath
    path[[length(path) + 1]] = c(x, y)
    reachableNodes[[length(reachableNodes) + 1]] <- list(x = x, y = y, cost = cost, heuristic = heuristic, totalCost = cost + heuristic, path = path)
  }
  path <- aStar$currPath
  x <- aStar$currX
  if(aStar$currY == 1) { # add above node only
    y <- aStar$currY + 1
    cost <- trafficMatrix$vroads[aStar$currX, aStar$currY] + aStar$costToHere
    heuristic <- aStar$heuristicMatrix[x, y]
    path[[length(path) + 1]] = c(x, y)
    reachableNodes[[length(reachableNodes) + 1]] <- list(x = x, y = y, cost = cost, heuristic = heuristic, totalCost = cost + heuristic, path = path)
  } else if (aStar$currY == n) { # add below node only
    y <- aStar$currY - 1
    cost <- trafficMatrix$vroads[aStar$currX, aStar$currY - 1] + aStar$costToHere
    heuristic <- aStar$heuristicMatrix[x, y]
    path[[length(path) + 1]] = c(x, y)
    reachableNodes[[length(reachableNodes) + 1]] <- list(x = x, y = y, cost = cost, heuristic = heuristic, totalCost = cost + heuristic, path = path)
  } else { # add above and below nodes
    y <- aStar$currY + 1
    cost <- trafficMatrix$vroads[aStar$currX, aStar$currY] + aStar$costToHere
    heuristic <- aStar$heuristicMatrix[x, y]
    path[[length(path) + 1]] = c(x, y)
    reachableNodes[[length(reachableNodes) + 1]] <- list(x = x, y = y, cost = cost, heuristic = heuristic, totalCost = cost + heuristic, path = path)
    y <- aStar$currY - 1
    cost <- trafficMatrix$vroads[aStar$currX, aStar$currY - 1] + aStar$costToHere
    heuristic <- aStar$heuristicMatrix[x, y]
    path <- aStar$currPath
    path[[length(path) + 1]] = c(x, y)
    reachableNodes[[length(reachableNodes) + 1]] <- list(x = x, y = y, cost = cost, heuristic = heuristic, totalCost = cost + heuristic, path = path)
  }
  return(reachableNodes)
}

# update current position to best node's position
expandBestNode <- function(aStar) {
  bestIndex <- which.min(sapply(aStar$frontier, function(frontierNode) frontierNode$totalCost))
  bestNode <- aStar$frontier[[bestIndex]]
  aStar$currX <- bestNode$x
  aStar$currY <- bestNode$y
  aStar$costToHere <- bestNode$cost
  aStar$currPath <- bestNode$path
  aStar$frontier[[bestIndex]] <- NULL # remove node from frontier
  return(aStar)
}

testDM(myFunction, verbose = 1, returnVec = FALSE, n = 500, seed = 21, timeLimit = 250)