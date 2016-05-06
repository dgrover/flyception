readFMF <- function(filepath, start=1, end=0, skip=0, crop=c(0,0,0,0), frames=NULL, getFrames=F){
  require(RImageBook)
  con <- file(filepath, open="rb")
  fmf_header <- readBin(con, "raw", 28)
  version <- raw2int(rev(fmf_header[1:4]))
  if(version != 1) stop("Only version 1 is supported")
  f_height <- raw2num(rev(fmf_header[5:8]))
  f_width <- raw2num(rev(fmf_header[9:12]))
  framesize <- f_width*f_height
  bytes_per_chunk <- raw2num(rev(fmf_header[13:20]))
  max_n_frames <- raw2num(rev(fmf_header[21:28]))
  if(max_n_frames == 0) max_n_frames <- round((file.info(filepath)$size - 28)/(bytes_per_chunk)) # may not be correct
  if(getFrames==T) {
    close(con)
    return(max_n_frames)
  }
  if(end==0 | end > max_n_frames) end <- max_n_frames
  if(is.null(frames)){
    nframes <- end - start + 1
  } else {
    if(max(frames) > max_n_frames) stop("Incorrect frames!")
    nframes <- length(frames)
    start <- frames[1]
    end <- tail(frames, n=1)
  }
  startpos <- 28 + 1 + (start-1)*bytes_per_chunk
  endpos <- 28 + 1 + end*bytes_per_chunk - 1
  seek(con, where = startpos - 1)
  
  if(is.null(frames)==F){
    imgdata <- raw(framesize*nframes)
    for(i in 1:length(frames)){
      if(i > 1){
        if(frames[i]==frames[i-1]){
          imgdata[(framesize*(i-1)+1):(framesize*i)]<- imgrawdata[9:bytes_per_chunk]
        } else {
          seek(con, where = (frames[i]-frames[i-1]-1)*bytes_per_chunk, origin="current")
          imgrawdata <- readBin(con, "raw", bytes_per_chunk)
          imgdata[(framesize*(i-1)+1):(framesize*i)]<- imgrawdata[9:bytes_per_chunk]
        }
      } else{
        imgrawdata <- readBin(con, "raw", bytes_per_chunk)
        imgdata[(framesize*(i-1)+1):(framesize*i)]<- imgrawdata[9:bytes_per_chunk]
      }
    }
    close(con)
    print(paste("Read specified frames of ", nframes, " frames", sep=""))
    rm(imgrawdata)
    array(as.integer(imgdata), dim=c(f_width, f_height, nframes))
  } else {
    if(skip==0){
      if(all.equal(crop, c(0,0,0,0))!=T){
        x1 <- crop[1]
        x2 <- crop[2]
        y1 <- crop[3]
        y2 <- crop[4]
        w <- x2-x1+1
        h <- y2-y1+1
        cropsize <- c(x2-x1+1)*c(y2-y1+1)
        imgdata <- raw(cropsize*nframes)
        for(j in 1:nframes){
          imgrawdata <- readBin(con, "raw", bytes_per_chunk)
          tmpmat <- matrix(imgrawdata[9:bytes_per_chunk], ncol=f_width)
          imgdata[(cropsize*(j-1)+1):(cropsize*j)] <-  as.vector(tmpmat[x1:x2,y1:y2])
        } 
        print(paste("Read ", nframes, " frames with cropping.", sep=""))
        close(con)
      } else {
        w <- f_width
        h <- f_height
        imgrawdata <- readBin(con, "raw", endpos-startpos+1)
        close(con)
        imgdata <- raw(framesize*nframes)
        for(i in 1:nframes){
          imgdata[(framesize*(i-1)+1):(framesize*i)]<- imgrawdata[((i-1)*bytes_per_chunk + 9):(i*bytes_per_chunk)]
        } 
        print(paste("Read ", start, "-", end, " of ", max_n_frames, " frames", sep=""))
        rm(imgrawdata)
      }
      array(as.integer(imgdata), dim=c(w, h, nframes))
    } else{
      nframes <- length(seq(1, nframes, skip+1))
      imgdata <- raw(framesize*nframes)
      for(i in 1:nframes){
        imgrawdata <- readBin(con, "raw", bytes_per_chunk)
        imgdata[(framesize*(i-1)+1):(framesize*i)]<- imgrawdata[9:bytes_per_chunk]
        seek(con, where = skip*bytes_per_chunk, origin="current")
      }
      close(con)
      print(paste("Read ", start, "-", end, " of ", max_n_frames, " frames skipping every ", skip, " frames", sep=""))
      rm(imgrawdata)
      array(as.integer(imgdata), dim=c(f_width, f_height, nframes))
    }
  }
}