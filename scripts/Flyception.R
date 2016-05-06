Flyception <- function(rdir, dir, prefix, autopos=T, video_out=F, mating=T, stimlen=200, 
                       stimplotlen=800, reuse=T, fmf2tif=T, fpsfl=100){
  # Version Flyception_04132016.R
  
  require(RImageBook)
  require(Rcpp)
  require(zoo)
  require(RNiftyReg)
  require(plotrix)
  require(ggplot2)
  require(plyr)
  require(reshape2)
  require(rlogging)
  source(paste0(rdir, "sfeatures.R"))
  source(paste0(rdir, "rollmeanimg.R"))
  source(paste0(rdir, "rollmedianimg.R"))
  source(paste0(rdir, "sweepC.R"))
  source(paste0(rdir, "readFMF2.R"))
  source(paste0(rdir, "plotter.R"))
  source(paste0(rdir, "drawtext.R"))
  source(paste0(rdir, "fmf2tif.R"))
  source(paste0(rdir, "pseudoColor3.R"))
  
  # Start logging
  SetLogFile(base.file=paste0(prefix, "_log.txt"), folder=dir)
  message(dir)
  
  # Load fluorescence movie and detect flash
  flfile <- paste0(dir, list.files(dir, pattern="ome\\.tif$"))
  message(sprintf("Reading %s", flfile))
  flimg <- readImage(flfile)
  message("Detecting flash in flview")
  flimgint <- colMeans(flimg, dim=2)   
  png(file=paste0(dir, prefix, "_flflash.png"), width=400, height=400)
  plot(flimgint)
  dev.off()
  flimgintdif <- diff(flimgint)
  flflashes <- which(flimgintdif > 0.04) + 1
  flimgflash <- min(flflashes)
  if(flimgflash==Inf) stop("Flash was not detected in flcamera.")
  nframesfl <- dim(flimg)[3]    
  flimgrt <- rotate(flip(flimg), -90)
  rm(flimg)
  message(paste0("Number of flash detected in flview: ", length(flflashes)))
  
  # Detect flash in flyview
  message("Detecting flash in flyview")
  if(file.exists(paste0(dir, prefix, "_fvimgsubint.RDS"))==T & reuse==T){
    message("Loading from RDS file")
    fvimgsubint <- readRDS(paste0(dir, prefix, "_fvimgsubint.RDS"))
  }else{
    fvimgsub1 <- readFMF(paste0(dir, list.files(dir, pattern="^fv.*fmf$")), crop=c(5,10,5,10))
    fvimgsub2 <- readFMF(paste0(dir, list.files(dir, pattern="^fv.*fmf$")), crop=c(220,225,220,225))
    fvimgsubint1 <- colMeans(fvimgsub1, dim=2)
    fvimgsubint2 <- colMeans(fvimgsub2, dim=2)
    rm(fvimgsub1)
    rm(fvimgsub2)
    fvimgsubint <- ifelse(fvimgsubint1 > fvimgsubint2, fvimgsubint1, fvimgsubint2)
    png(file=paste0(dir, prefix, "_fvflash.png"), width=400, height=400)
    plot(fvimgsubint)
    dev.off()
    saveRDS(fvimgsubint, file=paste0(dir, prefix, "_fvimgsubint.RDS"))
  }
  
  fvimgflash <- min(which(fvimgsubint > 135))
  if(fvimgflash==Inf) stop("Flash was not detected in fvcamera.")
  message(paste0("Number of flash detected in flyview: ", length(which(fvimgsubint > 135))))
  
  # Detect flash in arenaview
  message("Detecting flash in arenaview")
  if(file.exists(paste0(dir, prefix, "_avimgsubint.RDS"))==T & reuse==T){
    message("Loading from RDS file")
    avimgsubint <- readRDS(paste0(dir, prefix, "_avimgsubint.RDS"))
  }else{
    avimgsub <- readFMF(paste0(dir, list.files(dir, pattern="^av.*fmf$")), crop=c(5,10,5,10))
    avimgsubint <- colMeans(avimgsub, dim=2)
    rm(avimgsub)
    png(file=paste0(dir, prefix, "_avflash.png"), width=400, height=400)
    plot(avimgsubint)
    dev.off()
    saveRDS(avimgsubint, file=paste0(dir, prefix, "_avimgsubint.RDS"))
  }
  avimgflash <- min(which(avimgsubint > 100))
  if(avimgflash==Inf) stop("Flash was not detected in avcamera.")
  message(paste0("Number of flash detected in arenaview: ", length(which(avimgsubint > 100))))
  
  # Start time
  message("Analyzing metadata")
  metadata <-scan(paste0(dir, list.files(dir, pattern="metadata\\.txt$")), what=character(),sep="")
  log <- scan(paste0(dir, list.files(dir, pattern="fv-log-")), what=character(),sep="")
  avlog <- scan(paste0(dir, list.files(dir, pattern="av-log-")), what=character(),sep="")
  starttimefl <- metadata[which(metadata == "Time")[1]+2]
  
  # Exposure and binning
  exposure <- substr(metadata[which(metadata == "Exposure-ms")[1]+2], 1, nchar(metadata[which(metadata == "Exposure-ms")[1]+2])-1)
  binning <- metadata[which(metadata == "Binning")[1]+2]
  message(sprintf("flview exposure: %s", exposure))
  message(sprintf("flview binning: %s", binning))
  
  # Elapsed time (in ms) of each frame from the fluorescence camera relative to the flash
  elapsedtimefl <- metadata[grep("ElapsedTime-ms", metadata)+2]
  elapsedtimefl <- as.numeric(substr(elapsedtimefl, 1, nchar(elapsedtimefl)-1))
  fpsfl <- round(length(elapsedtimefl)/tail(elapsedtimefl, n=1)*1000, 2)
  message(paste("fluorescence camera:", fpsfl, "fps"))
  elapsedtimeflflash <- elapsedtimefl - elapsedtimefl[flimgflash]
  elapsedtimeflflashdiff <- diff(elapsedtimeflflash)
  
  # Elapsed time (in ms) of each frame from the fly view camera
  timestampusec <- as.numeric(log[grep("TimeStamp", log)+1])
  elapsedtimefv <- (timestampusec - timestampusec[1])/1000
  elapsedtimefvflash <- elapsedtimefv - elapsedtimefv[fvimgflash]
  fpsfv <- round(length(elapsedtimefv)/((tail(elapsedtimefv, n=1) - elapsedtimefv[1])/1000), 2)
  message(paste("flyview camera:", fpsfv, "fps")) 
  elapsedtimefvflashdiff <- diff(elapsedtimefvflash)
  # Only used for plotting purpose
  elapsedtime <- elapsedtimeflflash
  
  # Elapsed time (in ms) of each frame from the arenaview camera
  avtimestampcyclesec <- avlog[grep("TimeStamp", avlog)+1]
  avtimestampcyclesec <- as.numeric(avtimestampcyclesec)
  cnt <- 0
  avtimestampsec <- rep(0, length(avtimestampcyclesec))
  avtimestampsec[1] <- avtimestampcyclesec[1]
  for(t in 2:length(avtimestampsec)){
    if(avtimestampcyclesec[t-1]==127 & avtimestampcyclesec[t]==0) cnt <- cnt + 1
    avtimestampsec[t] <- avtimestampcyclesec[t] + 127*cnt
  }
  avtimestampsec <- avtimestampsec*1000
  avtimestampcnt <- avlog[grep("TimeStamp", avlog)+2]
  avtimestampmsec <- 1/8000*as.numeric(avtimestampcnt)*1000
  elapsedtimeav <- (avtimestampsec + avtimestampmsec) - (avtimestampsec[1] + avtimestampmsec[1])
  elapsedtimeavflash <- elapsedtimeav - elapsedtimeav[avimgflash]
  fpsav <- round(length(elapsedtimeav)/((tail(elapsedtimeav, n=1) - elapsedtimeav[1])/1000), 2)
  message(paste("arenaview camera:", fpsav, "fps")) 
  
  # Align frames between flyview and flview cameras
  message("Aligning frames between flyview and flview")
  if(file.exists(paste0(dir, prefix, "_frid.RDS"))==T & reuse==T){
    message("Loading RDS file")
    frid <- readRDS(paste0(dir, prefix, "_frid.RDS"))
  }else{
    frameratio <- round(fpsfv/fpsfl)
    message(paste0("fv/fl frame ratio: ", frameratio))
    # Hypothetical trigger
    frid <- seq(fvimgflash-(flimgflash-1)*frameratio, 
                fvimgflash+frameratio*(nframesfl-flimgflash), frameratio)
    # Interval between frames in ms
    framediff <- elapsedtimefvflashdiff
    # Convert ms to trigger count
    framediff[which(framediff<2)] <- 1
    framediff[which(framediff>2 & framediff<3)] <- 2
    framediff[which(framediff>3)] <- 3
    # Generate sequence of triggers for each frame
    fvfrsum <- cumsum(framediff)
    froffset <- fvfrsum[fvimgflash] - fvimgflash
    fvfrsum <- fvfrsum - froffset
    # Find matching and nearest frames to the hypothetical trigger
    frid2 <- match(frid, fvfrsum)
    frid2[which(is.na(frid2))] <- sapply(frid[which(is.na(frid2))], function(x) which.min(abs(fvfrsum-x)))
    frid <- unlist(frid2)
    # Check if two flashes match
    fvflashesfrid <- frid[flflashes]
    fvflashes <- which(fvimgsubint > 135)
    fvflashes
    fvflashesfrid
    message(paste0("Did flash frames match in flyview and flview? ", all.equal(fvflashesfrid, fvflashes)))
    if(length(flflashes)!=length(fvflashes)) {
      warning("Number of flash detected did not match between flyview and flview.")
    }
    # Load flyview camera images
    fvimgmaxfr <- readFMF(paste0(dir, list.files(dir, pattern="^fv.*fmf$")), getFrames=T)
    frid <- frid[which(frid<=fvimgmaxfr)]
    if(length(frid) < nframesfl){
      frid <- c(frid, rep(tail(frid, 1), nframesfl-length(frid)))
    }
    saveRDS(frid, paste0(dir, prefix, "_frid.RDS"))
  }
  fvimgl <- readFMF(paste0(dir, list.files(dir, pattern="^fv.*fmf$")), frames=frid)
  nframesfv <- dim(fvimgl)[3]
  
  # Align frames between flview and arenaview cameras
  message("Aligning frames between flview and arenaview")
  if(file.exists(paste0(dir, prefix, "_frida.RDS")) & reuse==T){
    message("Loading RDS file")
    frida <- readRDS(paste0(dir, prefix, "_frida.RDS"))
  }else{
    frameratio2 <- round(fpsav/fpsfl)
    message(paste0("av/fl frame ratio: ", frameratio2))
    # Hypothetical trigger
    frida <- seq(avimgflash-(flimgflash-1)*frameratio2, 
                 avimgflash+frameratio2*(nframesfl-flimgflash), frameratio2)
    # Check if two flashes match
    avflashesfridav <- frida[flflashes]
    avflashes <- which(avimgsubint > 100)
    message(paste0("Did flash frames match in arenaview and flview? ", all.equal(avflashesfridav, avflashes)))
    if(length(flflashes)!=length(avflashes)) {
      message("Number of flash detected did not match between areanview and flfiew.")
    }
  }
  
  # Calculating speed from trajectory
  fvtrj <- read.table(paste0(dir, list.files(dir, pattern="fv-traj-")))
  trj <- fvtrj[,c(2,3)]
  headpos <- fvtrj[,c(4,5)]
  distance <- trackDistance(trj)
  distance <- rollmedian(distance, k=5)
  speed <- rollsum(distance, k=200)/200*fpsfv
  error <- sqrt(diff(headpos[,1])^2 + diff(headpos[,2])^2)
  stimulus <- which(fvtrj[,10]==1)
  
  # Configure stimulus
  if(length(stimulus)!=0){
    fridstim <- sapply(stimulus, function(x) which.min(abs(frid-x)))
  } else {
    fridstim <- 6
  }
  
  # If flies are mating
  if(mating==T){
    trja <- read.table(paste0(dir, list.files(dir, pattern="av-traj-")), colClasses = "character")[,2:5]
    trja <- as.data.frame(sapply(trja,gsub,pattern="\\[",replacement=""), stringsAsFactors=F)
    trja <- as.data.frame(sapply(trja,gsub,pattern="\\]",replacement=""), stringsAsFactors=F)
    trja <- sapply(trja, as.numeric)
    flydist <- sqrt((trja[,1] - trja[,3])^2 + (trja[,2] - trja[,4])^2)
    if(nframesfl > length(flydist)){
      flydist <- c(flydist, rep(tail(flydist, 1), nframesfl-length(flydist)))
    }
    png(file=paste0(dir, prefix, "_flydist2.png"), width=400, height=400)
    plot(flydist, ylab="Distance between flies (mm)")
    dev.off()
    closefr <- which(flydist < 4)
    stimulusa <- c(closefr[1], closefr[which(diff(closefr)>stimlen)+1])
    if(is.na(stimulusa)){
      fridstim <- 6
    } else {
      fridstim <- sapply(stimulusa, function(x) which.min(abs(frida-x)))
      fridstim <- fridstim + 150
      fridstim[which(fridstim < 1)] <- 1
    }
    
  } else {
    trja <- read.table(paste0(dir, list.files(dir, pattern="av-traj-")), colClasses = "character")[,2:3]
    trja <- as.data.frame(sapply(trja,gsub,pattern="\\[",replacement=""), stringsAsFactors=F)
    trja <- as.data.frame(sapply(trja,gsub,pattern="\\]",replacement=""), stringsAsFactors=F)
    trja <- sapply(trja, as.numeric)
    flydist <- rep(0, nframesfl)
  }
  
  message(paste0("Stimuli were given at the following frames:"))
  message(fridstim)
  dfstim <- data.frame(flview=fridstim, flyview=frid[fridstim], arenaview=frida[fridstim])
  write.table(dfstim, paste0(dir, prefix, "_fridstim.txt"))
  
  # Load arenaview camera images and adjust frida
  avimgmaxfr <- readFMF(paste0(dir, list.files(dir, pattern="^av.*fmf$")), getFrames=T)
  frida <- frida[which(frida<=avimgmaxfr)]
  if(length(frida) < nframesfl){
    frida <- c(frida, rep(tail(frida, 1), nframesfl-length(frida)))
  }
  saveRDS(frida, paste0(dir, prefix, "_frida.RDS"))
  
  # Save arena view around the stimulus frames
  message("Saving arenaview around the stimulus frames")
  for(as in 1:length(fridstim)){
    Ffras <- fridstim[as]:(fridstim[as]+stimplotlen-1)
    Ffras <- Ffras[which(Ffras < length(frida))]
    avimglstim <- readFMF(paste0(dir, list.files(dir, pattern="^av.*fmf$")), frames=frida[Ffras])
    writeImage(avimglstim/255, file=paste0(dir, prefix, "_avimglstim_", as, "_fr_", frida[Ffras[1]], "-", frida[tail(Ffras, n=1)], ".tif"))
  }
  rm(avimglstim)
  
  # Detect window on the head
  message("Performing window detection...")
  if(file.exists(paste0(dir, prefix, "_fvimgbwbrfh.RDS"))==T & 
     file.exists(paste0(dir, prefix, "_ftrs.RDS"))==T & reuse==T){
    message("Loading RDS file")
    fvimgbwbrfh <- readRDS(paste0(dir, prefix, "_fvimgbwbrfh.RDS"))
    ftrs  <- readRDS(paste0(dir, prefix, "_ftrs.RDS"))
    
  }else{
    fvimgbw <- thresh(fvimgl, 20, 20, 0.1)
    writeImage(fvimgl[,,1]/255, file=paste0(dir, prefix, "_fvimgl.png"))
    writeImage(fvimgbw[,,1], file=paste0(dir, prefix, "_fvimgbw.png"))
    centermask <- drawCircle(matrix(0,dim(fvimgl)[1],dim(fvimgl)[2]), dim(fvimgl)[1]/2, dim(fvimgl)[2]/2, 100, col=1, fill=1)
    fvimgbwc <- ssweep(fvimgbw, centermask, op="*")
    writeImage(fvimgbwc[,,1], file=paste0(dir, prefix, "_fvimgbwc.png"))
    rm(fvimgbw)
    fvimgbd <- (255-fvimgl) > 180
    writeImage(fvimgbd[,,1], file=paste0(dir, prefix, "_fvimgbd.png"))
    fvimgbwhd <- fvimgbwc * fvimgbd
    writeImage(fvimgbwhd[,,1], file=paste0(dir, prefix, "_fvimgbwhd.png"))
    rm(fvimgbd)
    rm(fvimgbwc)
    kern1 <- makeBrush(3, shape="diamond")
    fvimgbwhdo <- opening(fvimgbwhd, kern1)
    rm(fvimgbwhd)
    writeImage(fvimgbwhdo[,,1], file=paste0(dir, prefix, "_fvimgbwhdo.png"))
    fvimgbwhdlb <- bwlabel(fvimgbwhdo)
    fvimgbwbrfh <- fillHull(fvimgbwhdlb)
    rm(fvimgbwhdo)
    
    # Calculate object size
    message("Calculating window size")
    ftrs <- sfeatures(rdir, fvimgbwbrfh, fvimgbwbrfh)
    maxobj <- lapply(ftrs, function(x) x[which(x[, 'm.pxs'] == max(x[, 'm.pxs'])),])
    nonmaxobjid <- lapply(ftrs, function(x) which(x[, 'm.pxs'] != max(x[, 'm.pxs'])))
    fvimgbwbrfh <- rmObjects(fvimgbwbrfh, nonmaxobjid) > 0
    kern1 <- makeBrush(3, shape="diamond")
    fvimgbwbrfh <- dilate(fvimgbwbrfh, kern1)
    saveRDS(fvimgbwbrfh, paste0(dir, prefix, "_fvimgbwbrfh.RDS"))
    saveRDS(ftrs, paste0(dir, prefix, "_ftrs.RDS"))
    writeImage(fvimgbwbrfh[,,1], file=paste0(dir, prefix, "_fvimgbwbrfh.png"))
    rm(fvimgbwhdlb)
  }
  
  ftrs <- sfeatures(rdir, fvimgbwbrfh, fvimgbwbrfh)
  maxobj <- lapply(ftrs, function(x) x[which(x[, 'm.pxs'] == max(x[, 'm.pxs'])),])
  maxobj[sapply(maxobj, length)==0]<-NA
  objsize <- unlist(lapply(maxobj, function(x) x[c('m.pxs')]))
  objdist <- unlist(lapply(maxobj, function(x) sqrt((x['m.x']-dim(fvimgl)[1]/2)^2 + (x['m.y']-dim(fvimgl)[2]/2)^2)))
  png(file=paste0(dir, prefix, "_objsize.png"), width=400, height=400)
  plot(objsize)
  dev.off()
  png(file=paste0(dir, prefix, "_objdist.png"), width=400, height=400)
  plot(objdist)
  dev.off()
  png(file=paste0(dir, prefix, "_error.png"), width=400, height=400)
  plot(error)
  dev.off()
  objsizemedian <- median(objsize, na.rm=T)
  message(sprintf("Median window size is %.1f", objsizemedian))
  objsizemad <- mad(objsize, na.rm=T)
  goodsizefr <- which(objsize > objsizemedian - 4*objsizemad & objsize < objsizemedian + 4*objsizemad)
  message(sprintf("Good size frame: %d", length(goodsizefr)))
  message(sprintf("The following frames have too big or too small window: %s", paste((1:length(objsize))[-goodsizefr], collapse=" ")))
  goodposfr <- which(objdist < 20)
  message(sprintf("The following frames have window too far from the center: %s", paste((1:length(objsize))[-goodposfr], collapse=" ")))
  gooderrorfr <- which(error[frid] < 5)
  message(sprintf("The following frames have too large motion: %s", paste((1:length(objsize))[-gooderrorfr], collapse=" ")))
  goodfr <- intersect(gooderrorfr, goodsizefr)
  message(sprintf("Good size and good error frame: %d", length(goodfr)))
  goodfr <- intersect(goodposfr, goodfr)  
  message(sprintf("Good size, good error, good position frame: %d", length(goodfr)))
  
  # Detect blurriness
  message("Detecting blurriness...")
  if(file.exists(paste0(dir, prefix, "_quantcnt.RDS"))==T & reuse==T){
    message("Loading RDS file")
    quantcnt <- readRDS(paste0(dir, prefix, "_quantcnt.RDS"))
  }else{
    
    LoG <- function(x,y,s){
      fn <- function(x, y, s) 1/(pi*s^4)*((x^2 + y^2)/(2 * s^2)-1)*exp(-(x^2 + y^2)/(2*s^2))
      x <- seq(-floor(x/2), floor(x/2), len = x)
      y <- seq(-floor(y/2), floor(y/2), len = y)
      w <- outer(x, y, fn, s)
      w
    }
    LoGkern <- round(LoG(9,9,1.4)*428.5)
    fvimgllog <- filter2(fvimgl, LoGkern)
    centermask <- drawCircle(fvimgl[,,1]*0, dim(fvimgl)[1]/2, dim(fvimgl)[2]/2, 100, col=1, fill=T)
    fvimgcntlog <- sweep(fvimgllog, 1:2, centermask, FUN="*")
    rm(fvimgllog)
    quantcnt <- apply(fvimgcntlog, 3, function(x) quantile(x, 0.9))
    png(file=paste0(dir, prefix, "_cntLoG_quant.png"), width=800, height=800)
    plot(quantcnt, ylim=c(0, max(quantcnt)))
    dev.off()
    rm(fvimgcntlog)
    saveRDS(quantcnt, paste0(dir, prefix, "_quantcnt.RDS"))
    
  }
  
  sharpfr <- which(quantcnt > 150)
  blurfr <- which(quantcnt <= 150)
  message(sprintf("The following frames are blurry: %s", paste(blurfr, collapse=" ")))
  goodfr <- intersect(goodfr, sharpfr)
  message(sprintf("Good size, good error, good position, good focus frame: %d", length(goodfr)))
  
  # Position calibration
  zoom <- 0.85/as.numeric(binning)  
  
  # Manual position calibration with fly contour during flash
  if(autopos==F){
    center <- c(-2, 10) # x positive left, y positive up
    flref <- normalize(flimgrt[,,flflashes[1]])
    fvref <- fvimgl[,,flflashes[1]]/255
    display(flref)
    display(fvref)
    fvrefrs <- resize(fvref, dim(fvref)[1]*zoom)
    display(fvrefrs)
    flrefpad <- fvrefrs*0
    flrefpad[round((dim(fvrefrs)[1]-dim(flimgrt)[1])/2):
               (round((dim(fvrefrs)[1]-dim(flimgrt)[1])/2)+dim(flimgrt)[1]-1),
             round((dim(fvrefrs)[2]-dim(flimgrt)[2])/2):
               (round((dim(fvrefrs)[2]-dim(flimgrt)[2])/2)+dim(flimgrt)[2]-1)] <- flref
    flrefpadmv <- translate(flrefpad, center)
    display(flrefpadmv)
    display(normalize(fvrefrs + flrefpadmv))
    writeImage(normalize(fvrefrs + flrefpadmv), file=paste0(dir, prefix, "_aligned.png"))
    
  } else {
    # Automated position calibration using template matching
    message("Automatically aligning two cameras...")
    flref <- normalize(flimgrt[,,flflashes[1]])
    fvref <- fvimgl[,,flflashes[1]]/255
    writeImage(flref, file=paste0(dir, prefix, "_flref.png"))
    writeImage(fvref, file=paste0(dir, prefix, "_fvref.png"))
    fvrefrs <- resize(fvref, dim(fvref)[1]*zoom)
    fncc <- FNCC(fvrefrs, flref)
    maxpeak <- which(fncc==max(fncc), arr.ind=TRUE)
    fnccres <- fvrefrs
    fnccres[(maxpeak[1]):(maxpeak[1]+nrow(flref)-1),
            (maxpeak[2]):(maxpeak[2]+ncol(flref)-1)] <- flref
    centerx <- (maxpeak[1] + round(nrow(flref)/2)) - round(dim(fvrefrs)[1]/2)
    centery <- (maxpeak[2] + round(ncol(flref)/2)) - round(dim(fvrefrs)[2]/2)
    center <- c(centerx, centery)
    flrefpad <- fvrefrs*0
    flrefpad[round((dim(fvrefrs)[1]-dim(flimgrt)[1])/2):
               (round((dim(fvrefrs)[1]-dim(flimgrt)[1])/2)+dim(flimgrt)[1]-1),
             round((dim(fvrefrs)[2]-dim(flimgrt)[2])/2):
               (round((dim(fvrefrs)[2]-dim(flimgrt)[2])/2)+dim(flimgrt)[2]-1)] <- flref
    flrefpadmv <- translate(flrefpad, center)
    writeImage(normalize(fvrefrs + flrefpadmv), file=paste0(dir, prefix, "_aligned.png"))
    
  }
  
  # Image registration for both fvimg and flimg
  message("Performing image registration...")
  if(file.exists(paste0(dir, prefix, "_regimgi.RDS"))==T & 
     file.exists(paste0(dir, prefix, "_regresi.RDS"))==T & reuse==T){
    message("Loading RDS file")
    regimgi <- readRDS(paste0(dir, prefix, "_regimgi.RDS"))
    regresi <- readRDS(paste0(dir, prefix, "_regresi.RDS"))
  }else{
    
    edgepos <- fvtrj[,c(6,7)]
    angles <- atan2((headpos - edgepos)[,1], (headpos - edgepos)[,2])
    # Prepare fvimg for registration
    fvimgli <- resize(255 - fvimgl, dim(fvimgl)[1]*zoom)
    # replace flash frames with one previous frame
    fvimgli[,,flflashes] <- fvimgli[,,(flflashes-1)] 
    centermask <- drawCircle(matrix(0,dim(fvimgli)[1],dim(fvimgli)[2]), dim(fvimgli)[1]/2, dim(fvimgli)[2]/2, dim(fvimgli)[1]/2-1, col=1, fill=1)
    # Create first image, which will be the target in registration
    fvimgrt1sti <- rotate(fvimgli[,,1], angles[frid[1]]*180/pi, output.dim=dim(fvimgli)[1:2])
    # Build affine matrix for rotation
    aff <- list()
    for(a in 1:dim(fvimgli)[3]){
      aff[[a]] <- buildAffine(angles=c(0,0, -angles[frid[a]]), source=fvimgli[,,1], anchor="center")
    }
    # Run image registration using the initial angle
    regresi <- list()
    for(rg in 1:dim(fvimgli)[3]){
      regresi[[rg]] <- niftyreg(fvimgli[,,rg], fvimgrt1sti,
                                init=aff[[rg]], scope="rigid", symmetric=F)
    }
    regimgi <- array(sapply(regresi, function(x) x$image), dim=dim(fvimgli))
    regimgi[which(is.na(regimgi)==T)] <- 0
    saveRDS(regimgi, paste0(dir, prefix, "_regimgi.RDS"))
    saveRDS(regresi, paste0(dir, prefix, "_regresi.RDS"))
    writeImage((255-regimgi)/255, file=paste0(dir, prefix, "_regimgi.tif"))
    rm(fvimgli)
  }
  
  # Apply affine transformation to flimg
  message("Transforming flimg...")
  if(file.exists(paste0(dir, prefix, "_flimgreg.RDS"))==T & reuse==T){
    message("Loading RDS file")
    flimgreg <- readRDS(paste0(dir, prefix, "_flimgreg.RDS"))
    
  }else{
    
    # Pad flimgrt to match the size of fvimg
    flimgpad <- regimgi*0
    flimgpad[round((dim(regimgi)[1]-dim(flimgrt)[1])/2):
               (round((dim(regimgi)[1]-dim(flimgrt)[1])/2)+dim(flimgrt)[1]-1),
             round((dim(regimgi)[2]-dim(flimgrt)[2])/2):
               (round((dim(regimgi)[2]-dim(flimgrt)[2])/2)+dim(flimgrt)[2]-1),
             1:dim(flimgrt)[3]] <- flimgrt
    flimgpadmv <- translate(flimgpad, center)
    flimgrgres <- list()
    for(app in 1:dim(fvimgl)[3]){
      flimgrgres[[app]] <- applyTransform(forward(regresi[[app]]), flimgpadmv[,,app])
    }
    flimgreg <- array(unlist(flimgrgres), dim=dim(flimgpadmv))
    writeImage(flimgreg, file=paste0(dir, prefix, "_flimgreg.tif"))
    rm(flimgrgres)
    rm(flimgpad)
    rm(flimgpadmv)
    saveRDS(flimgreg, paste0(dir, prefix, "_flimgreg.RDS"))
    
  }
  
  # Apply affine transformation to window mask
  
  message("Transforming window images...")
  if(file.exists(paste0(dir, prefix, "_fvimgbwbrfhregimg.RDS"))==T & reuse==T){
    message("Loading RDS file")
    fvimgbwbrfhregimg <- readRDS(paste0(dir, prefix, "_fvimgbwbrfhregimg.RDS"))
  }else{
    
    fvimgbwbrfhrs <- resize(fvimgbwbrfh, dim(fvimgl)[1]*zoom)
    rm(fvimgbwbrfh)
    fvimgbwbrfhreg <- list()
    for(app in 1:dim(fvimgl)[3]){
      fvimgbwbrfhreg[[app]] <- applyTransform(forward(regresi[[app]]), fvimgbwbrfhrs[,,app])
    }
    fvimgbwbrfhregimg <- array(unlist(fvimgbwbrfhreg), dim=dim(fvimgbwbrfhrs))
    fvimgbwbrfhregimg <- fvimgbwbrfhregimg >= 0.5
    writeImage((255-regimgi)/255*(1-fvimgbwbrfhregimg) + normalize(flimgreg), file=paste0(dir, prefix, "_fvfvwindflregimg.tif"))
    rm(fvimgbwbrfhrs)
    rm(fvimgbwbrfhreg)
    rm(regresi)
    saveRDS(fvimgbwbrfhregimg, paste0(dir, prefix, "_fvimgbwbrfhregimg.RDS"))
    
  }
  
  # Detect window position after registration
  ftrs2 <- sfeatures(rdir, fvimgbwbrfhregimg, fvimgbwbrfhregimg)
  maxobj2 <- lapply(ftrs2, function(x) x[which(x[, 'm.pxs'] == max(x[, 'm.pxs'])),])
  maxobj2[sapply(maxobj2, length)==0]<-NA
  objsize2 <- unlist(lapply(maxobj2, function(x) x[c('m.pxs')]))
  objdist2 <- unlist(lapply(maxobj2, function(x) sqrt((x['m.x']-dim(fvimgbwbrfhregimg)[1]/2)^2 + (x['m.y']-dim(fvimgbwbrfhregimg)[2]/2)^2)))
  objx <- unlist(lapply(maxobj2, function(x) x[c('m.x')]))
  objy <- unlist(lapply(maxobj2, function(x) x[c('m.y')]))
  error2 <- sqrt(diff(objx)^2 + diff(objy)^2)
  nomotion <- which(error2 < 1)
  message(sprintf("The following frames have too big motion after registration: %s", paste((1:length(error2))[-nomotion], collapse=" ")))
  goodfr <- intersect(nomotion, goodfr)
  message(sprintf("Good size, good error, good position, good focus, good registration frame: %d", length(goodfr)))
  message(sprintf("The following frames have passed size, centering, motion, and sharpness filters: %s", paste(goodfr, collapse=" ")))
  
  # Calculate fluorescence intensity in the brain window
  message("Measuring fluorescence intensity...")
  if(file.exists(paste0(dir, prefix, "_intensity_br.RDS"))==T &
     file.exists(paste0(dir, prefix, "_intensity_wh.RDS"))==T & reuse==T){
    message("Using RDS file")
    intensity_br <- readRDS(paste0(dir, prefix, "_intensity_br.RDS"))
    intensity_wh <- readRDS(paste0(dir, prefix, "_intensity_wh.RDS"))
  }else{
    
    intensity_br <- colSums(fvimgbwbrfhregimg*flimgreg, dims=2)/as.integer(objsize)
    intensity_wh <- colSums(flimgreg, dims=2)
    saveRDS(intensity_br, paste0(dir, prefix, "_intensity_br.RDS"))
    saveRDS(intensity_wh, paste0(dir, prefix, "_intensity_wh.RDS"))
    
  }
  intensity <- na.approx(intensity_br)
  
  # Plot delta F over F0
  message("Creating dF/F0 plots")
  for(fintfr in 1:length(fridstim)){
    Fintfr <- fridstim[fintfr]:(fridstim[fintfr]+stimplotlen-1)
    Fintfr <- Fintfr[which(Fintfr < nframesfl)]
    Fintfrg <- intersect(goodfr, Fintfr)
    Fintfr[!Fintfr%in%goodfr] <- NA
    F0int <- mean(intensity[Fintfrg[1:5]])
    deltaFint <- intensity[Fintfr] - F0int
    dFF0int <- deltaFint/F0int * 100
    dat <- data.frame(x=(1:length(dFF0int)), y=dFF0int, d=flydist[fridstim[fintfr]:(fridstim[fintfr]+stimplotlen-1)])
    p <- ggplot(data=dat, aes(x=x, y=y)) +
      geom_smooth(method="loess", span = 0.4, level=0.95) +
      ylim(-5, 10) +
      geom_line(data=dat, aes(x=x, y=d))
    ggsave(filename = paste0(dir, prefix, "_dFF0int_", fintfr, ".pdf"), width = 8, height = 8)
  }
  
  # Create delta F over F0 pseudocolor representation only for good frames
  message("Calculating dF/F0 images...")
  for(ffr in 1:length(fridstim)){
    Ffr <- fridstim[ffr]:(fridstim[ffr]+stimplotlen-1)
    Ffr <- Ffr[which(Ffr < nframesfl)]
    oFfr <- Ffr
    Ffr <- intersect(goodfr, Ffr)
    wFfr <- which(oFfr%in%Ffr)
    dFfr <- data.frame(ori=wFfr, Ffr=1:length(Ffr), fl=Ffr, fv=frid[Ffr], av=frida[Ffr])
    write.table(dFfr, file=paste0(dir, prefix, "_dFfr_", ffr, ".txt"), row.names=F)
    
    Fmean <- rollmeanimg(flimgreg[,,Ffr], 5)
    # Fmedian <- rollmedianimg(flimgreg[,,Ffr], 5)
    F0 <- rowMeans(flimgreg[,,Ffr[1:5]], dims=2)
    deltaF <- ssweep(Fmean, F0, op="-")
    dFF0 <- ssweep(deltaF, 1/F0, op="*")
    dFF0[is.na(dFF0)] <- 0
    dFF0[is.infinite(dFF0)] <- 0
    dFF0masked <- fvimgbwbrfhregimg[,,Ffr]*dFF0
    dFF0maskedpos <- dFF0masked * 100 # Convert to %
    dFF0maskedpos[which(dFF0maskedpos < 0)] <- 0
   
    colmax <- median(apply(dFF0maskedpos, 3, max))
    colmax <- 100
    dFF0maskedpos <- medianFilter(dFF0maskedpos/colmax, 3) # median filter cuts > 1
    dFF0fin <- array(0, dim=c(dim(fvimgbwbrfhregimg)[c(1,2)], 3, length(Ffr)))
    for(cfr in 1:length(Ffr)){
      dFF0fin[,,,cfr] <- pseudoColor3(dFF0maskedpos[,,cfr], 64, 128)
    }
    dFF0fin <- Image(dFF0fin, colormode="Color")
    message(sprintf("Pseudocolor range for ffr=%d is 20 to %d", ffr, colmax))
    
    # Use window size for filtering
    F0size <- mean(objsize[Ffr[1]:(Ffr[1]+4)])
    dFF0size <- which(objsize[Ffr] > (F0size - 50) & objsize[Ffr] < (F0size + 50))
    
    # Use focus for filtering
    F0focus <- mean(quantcnt[Ffr[1]:(Ffr[1]+4)])
    dFF0focus <- which(quantcnt[Ffr] > (F0focus - 20) & quantcnt[Ffr] < (F0focus + 20))
    
    # Use both window size and focus for filtering
    dFF0size_focus <- intersect(dFF0size, dFF0focus)
    write.table(dFF0size_focus, file=paste0(dir, prefix, "_dFF0size_focus_", ffr, ".txt"))
    writeImage((100*Fmean)^2, file=paste0(dir, prefix, "_Fmean_", ffr, ".tif"))
    dFF0finmask <- array(0, dim=c(dim(fvimgbwbrfhregimg)[c(1,2)], 3, length(Ffr)))
    dFF0finmask[,,1,] <- fvimgbwbrfhregimg[,,Ffr]
    dFF0finmask[,,2,] <- fvimgbwbrfhregimg[,,Ffr]
    dFF0finmask[,,3,] <- fvimgbwbrfhregimg[,,Ffr]
    dFF0regimg <- array(0, dim=c(dim(fvimgbwbrfhregimg)[c(1,2)], 3, length(Ffr)))
    dFF0regimg[,,1,] <- 255-regimgi[,,Ffr]
    dFF0regimg[,,2,] <- 255-regimgi[,,Ffr]
    dFF0regimg[,,3,] <- 255-regimgi[,,Ffr]
    dFF0finmaskfly <- Image(dFF0fin*dFF0finmask+dFF0regimg/255, colormode="Color")
    writeImage(dFF0finmaskfly[,,,dFF0size_focus], bits.per.sample = 8, 
               file=paste0(dir, prefix, "_dFF0finmaskfly_sizefocus_", ffr, ".tif"))
    writeImage(dFF0finmaskfly, bits.per.sample = 8, 
               file=paste0(dir, prefix, "_dFF0finmaskfly_", ffr, ".tif"))
    
    # Uncomment for measuring dF/F in ROI
#     measureROI <- function(img, x, y, w, h){
#       ROI <- img[x:(x+w-1),y:(y+h-1),]
#       meanint <- colMeans(ROI, dim=2)
#       return(meanint)
#     }
#     leftROI <- measureROI(Fmean, 80, 96, 10, 10)
#     rightROI <- measureROI(Fmean, 110, 97, 10, 10)
#     ROI <- (leftROI + rightROI)/2
#       
#     ROIF0 <- mean(ROI[1:5])
#     deltaROIF <- ROI - ROIF0
#     ROIdFF0 <- deltaROIF/ROIF0 * 100
#     dat <- data.frame(x=wFfr, y=ROIdFF0, d=flydist[frida[Ffr]])
#     p <- ggplot(data=dat, aes(x=x, y=y)) +
#       geom_smooth(method="loess", span = 0.4, level=0.95) +
#       ylim(-5, 10) +
#       geom_line(data=dat, aes(x=x, y=d))
#     ggsave(filename = paste0(dir, prefix, "_dFF0int_ROI_", ffr, ".pdf"), width = 8, height = 8)
#     
    # Save videos of stimulus frames of flyview and arenaview
    writeImage((fvimgl[,,Ffr])/255, file=paste0(dir, prefix, "_fvimgl_stim_", ffr, ".tif"))
    avimglstim <- readFMF(paste0(dir, list.files(dir, pattern="^av.*fmf$")), frames=frida[Ffr])
    writeImage(avimglstim/255, file=paste0(dir, prefix, "_avimglstim_", ffr, ".tif"))
    
  }

  rm(dFF0)
  rm(dFF0masked)
  rm(dFF0maskedpos)
  rm(dFF0fin)
  rm(dFF0regimg)
  rm(dFF0finmask)
  rm(fvimgbwbrfhregimg)
  
  # Create trajectory of the flies at the time of stimulus delivery
  message("Creating trajectory of the flies...")
  for(tj in 1:length(fridstim)){
    Ffr <- fridstim[tj]:(fridstim[tj]+stimplotlen-1)
    pdf(file= paste0(dir, prefix, "_trackResult_", tj, ".pdf"), width = 4.4, height = 4, bg = "white")
    par(plt = c(0, 1, 0, 1), xaxs = "i", yaxs = "i")
    plot(trja[frida[Ffr[1]:(Ffr[1]+stimlen-1)],1]*10, -trja[frida[Ffr[1]:(Ffr[1]+stimlen-1)],2]*10, 
         type = "l", lty = 1, pch=tj, col="red",
         axes = F, xlim = c(-240, 240), ylim = c(-220, 220))
    par(new=T)
    plot(trja[frida[(Ffr[1]+stimlen):tail(Ffr, n=1)],1]*10, -trja[frida[(Ffr[1]+stimlen):tail(Ffr, n=1)],2]*10, 
         type = "l", lty = 2, pch=tj, col="red",
         axes = F, xlim = c(-240, 240), ylim = c(-220, 220))
    
    if(mating==T){
      par(new=T)
      plot(trja[frida[Ffr[1]:(Ffr[1]+stimlen-1)],3]*10, -trja[frida[Ffr[1]:(Ffr[1]+stimlen-1)],4]*10, 
           type = "l", lty = 1, pch=tj, col="blue",
           axes = F, xlim = c(-240, 240), ylim = c(-220, 220))
      par(new=T)
      plot(trja[frida[(Ffr[1]+stimlen):tail(Ffr, n=1)],3]*10, -trja[frida[(Ffr[1]+stimlen):tail(Ffr, n=1)],4]*10, 
           type = "l", lty = 2, pch=tj, col="blue",
           axes = F, xlim = c(-240, 240), ylim = c(-220, 220))
    }
    par(new=T)
    draw.ellipse(0,0,11.0795*20,10*20)
    dev.off()
    
  }
  
  rm(fvimgl)
  rm(flimgrt)
  rm(regimgi)
  
  # Plot intensity, speed, window size, etc. for both all frames and good frames
  message("Plotting results...")
  source("C:/Users/GreenspanLab/Dropbox/R/plotter.R")
  plotter(dir, prefix, intensity*16384, speed, error, objsize, quantcnt, 
          elapsedtime, elapsedtimefvflash, elapsedtimeavflash, exposure, binning, fpsfl, 
          starttimefl, type="all", stim = fridstim, stimlen = stimlen, flydist=flydist)
  intensity_g <- intensity
  intensity_g[-goodfr] <- NA
  speed_g <- speed
  speed_g[-frid[goodfr]] <- NA
  error_g <- error
  error_g[-frid[goodfr]] <- NA
  objsize_g <- objsize
  objsize_g[-goodfr] <- NA
  quant_g <- quantcnt
  quant_g[-goodfr] <- NA
  flydist_g <- flydist
  flydist_g[-goodfr] <- NA
  elapsedtime_g <- elapsedtime
  elapsedtime_g[-goodfr] <- NA
  elapsedtimefvflash_g <- elapsedtimefvflash
  elapsedtimefvflash_g[frid[-goodfr]] <- NA
  elapsedtimeavflash_g <- elapsedtimeavflash
  elapsedtimeavflash_g[frida[-goodfr]] <- NA
  plotter(dir, prefix, intensity_g, speed_g, error_g, objsize_g, quant_g, 
          elapsedtime_g, elapsedtimefvflash_g, elapsedtimeavflash_g, exposure, binning, fpsfl, 
          starttimefl, type="goodfr", stim = fridstim, stimlen = stimlen, flydist=flydist_g)
  
  if(video_out==T){
    
    dir.create(paste0(dir, "tmpimgs")) 
    for(k in 1:dim(dFF0finmaskfly)[4]){
      writeImage(dFF0finmaskfly[,,,k], file=paste0(dir, "tmpimgs/", prefix, "_", formatC(k,width=5,flag="0"), ".tiff"))
    }
    rm(arr)
    
    for(j in 1:nframesfl){
      png(file=paste0(dir, "tmpimgs/", prefix, "_intensity_", formatC(j,width=5,flag="0"), ".png"), width=400, height=250)
      par(mar=c(3,3,1,3))
      plot(elapsedtime[1:j], intensity[1:j], type="l", xlim=c(min(elapsedtime), max(elapsedtime)), 
           ylim=c(0, min(40, max(intensity, na.rm=T))), xlab=NA, ylab=NA, col='blue')
      par(new=T)
      id <- which(elapsedtimefvflash <= elapsedtime[j])
      plot(elapsedtimefvflash[id], speed[id], xlim=c(min(elapsedtime), max(elapsedtime)),
           axes=F, type='l', xlab=NA, ylab=NA, col="red", ylim=c(0, 40))
      axis(4)
      mtext(paste("Elapsed time (ms) from", starttimefl), side=1, line=2)
      mtext(paste("Fluorescence intensity (A.U.)"), side=2, line=2)
      mtext(paste("Fly speed (mm/s)"), side=4, line=2)
      legend("topright", inset=.05, c("Fluorescence","Speed"), col=c("blue", "red"), lty=1)
      title(main=paste0("Exposure: ", exposure, " ms, ", "Binning: ", binning, ", ", "fps: ", fpsfl))
      dev.off() 
    }
    
    cmd1 <- paste0("ffmpeg -f image2 -r ", fpsfl, " -i ", dir, "tmpimgs/", prefix, "_intensity_%05d.png", 
                   " -pix_fmt yuv444p -y -b 800k ", dir, prefix, "_plot.mp4")
    system(cmd1, ignore.stderr= T, show.output.on.console=F)
    cmd2 <- paste0("ffmpeg -f image2 -r ", fpsfl, " -i ", dir, "tmpimgs/", prefix, "_%05d.tiff", 
                   " -pix_fmt yuv444p -y -b 800k ", dir, prefix, "_video.mp4")
    system(cmd2, ignore.stderr= T, show.output.on.console=F)
    unlink(paste0(dir, "tmpimgs/*"))
    
    cmd3 <- paste0("ffmpeg -i ", dir, prefix, "_video.mp4 -i ", dir, prefix, 
                   "_plot.mp4 -filter_complex \"[0]pad=525:ih[bg]; [bg][1]overlay=125\" -pix_fmt yuv444p -y -b 800k ", dir, prefix, "_output.mp4")
    system(cmd3, ignore.stderr= T, show.output.on.console=F)
    
  }
  
  rm(dFF0finmaskfly)
  
  # Convert fmf to tif format
  if(fmf2tif==T){
    fmf2tif(paste0(dir, list.files(dir, pattern="^fv.*fmf$")), skip=10)
    fmf2tif(paste0(dir, list.files(dir, pattern="^av.*fmf$")), skip=2)
  }
  
  message("Finished processing!")
  gc()
}
