pseudoColor3 <- function (x, min = 0, max = 255) 
{
  x <- x * 255 + 1
  up <- seq(0, 255, by = round(256 * 4/(max - min + 1)))
  down <- seq(255, 0, by = -round(256 * 4/(max - min + 1)))
  half <- round((max - min - 1)/2)
  quat <- round((max - min - length(down) - 1)/2)
  if (max == 255) {
    lut.r <- c(seq(0, min, by = 1), rep(0, half), up, 
               rep(255, (max - min - half - length(up))))
    x.r <- matrix(lut.r[x], nrow(x), ncol(x))
    lut.g <- c(seq(0, min, by = 1), up, rep(255, (max - min - length(up) * 2)), down)
    x.g <- matrix(lut.g[x], nrow(x), ncol(x))
    lut.b <- c(seq(0, min, by = 1), rep(255, quat), down, 
               rep(0, (max - min - length(down) - quat)))
    x.b <- matrix(lut.b[x], nrow(x), ncol(x))
  }
  else {
    lut.r <- c(seq(0, min, by = 1), rep(0, half), up, 
               rep(255, (max - min - half - length(up) - 1)), rep(255, 255 - max))
    x.r <- matrix(lut.r[x], nrow(x), ncol(x))
    lut.g <- c(seq(0, min, by = 1), up, 
               rep(255, (max - min - length(up) * 2 - 1)), down, rep(0, 255 - max))
    x.g <- matrix(lut.g[x], nrow(x), ncol(x))
    lut.b <- c(seq(0, min, by = 1), rep(255, quat), down, 
               rep(0, (max - min - length(down) - 1 - quat)), rep(0, 255 - max))
    x.b <- matrix(lut.b[x], nrow(x), ncol(x))
  }
  x.rgb <- rgbImage(x.r/255, x.g/255, x.b/255)
}