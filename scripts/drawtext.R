drawtext <- function(img, text, position=NULL, col=NULL, font=NULL){
  bimg <- E2b(img)
  tmp <- tempfile(fileext=".png")
  if(is.null(position)) position <- c(dim(bimg)[1]/2, 15)
  png(file=tmp, width=dim(bimg)[1], height=dim(bimg)[2])
  par(plt=c(0, 1, 0, 1), xaxs="i", yaxs="i")
  plot(bimg)
  text(position[1], position[2], labels=text, col=col, font=font)
  dev.off()
  oimg <- readImage(tmp)
  unlink(tmp)
  oimg
}