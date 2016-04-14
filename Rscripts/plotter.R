plotter <- function(dir, sample, intensity, speed, error, objsize, quant, flydist, elapsedtime, elapsedtimefvflash, elapsedtimeavflash, exposure, binning, fpsfl, starttimefl, type="all", stim, stimlen){
  
  # Plot speed, fluorescence, and tracking error
  pdf(file=paste0(dir, sample, "_intensity_speed_error_", type, ".pdf"), width=8, height=6)
  par(mar=c(5,4,3,4))
  plot(elapsedtime, intensity, type="l", xlim=c(min(elapsedtime, na.rm=T), max(elapsedtime, na.rm=T)), 
       ylim=c(0, median(intensity, na.rm=T)*1.5), xlab=paste("Elapsed time (ms) from", starttimefl), 
       ylab=paste("Fluorescence intensity (A.U.)"), col='blue')
  par(new=T)
  plot(elapsedtimeavflash, flydist, type="l", xlim=c(min(elapsedtime, na.rm=T), max(elapsedtime, na.rm=T)), ylim=c(0, 10), 
       axes=F, xlab=NA, ylab=NA, col='black')
  par(new=T)
  plot(elapsedtimefvflash[1:length(speed)], speed, xlim=c(min(elapsedtime, na.rm=T), max(elapsedtime, na.rm=T)),
       axes=F, type='l', xlab=NA, ylab=NA, col="red", ylim=c(0, 40))
  rect(elapsedtime[stim], 0, elapsedtime[stim + stimlen], 60, col=rgb(red=0.2, green=0.2, blue=0.2, alpha=0.2), border = NA)
  axis(4)
  mtext(paste("Fly speed (mm/s), Tracking error (px)"), side = 4, line=3)
  #legend("topright", inset=.05, c("Fluorescence","Speed", "Error", "Distance"), col=c("blue", "red", "green", "black"), lty = 1)
  #title(main=paste0("Exposure: ", exposure, " ms, ", "Binning: ", binning, ", ", "fps: ", fpsfl))
  dev.off()
  
  # Plot fluorescence and tracking error
  png(file=paste0(dir, sample, "_intensity_error_", type, ".png"), width=800, height=600)
  par(mar=c(5,4,3,4))
  plot(elapsedtime, intensity, type="l", xlim=c(min(elapsedtime, na.rm=T), max(elapsedtime, na.rm=T)), 
       ylim=c(0, median(intensity, na.rm=T)*3), xlab=paste("Elapsed time (ms) from", starttimefl), 
       ylab=paste("Fluorescence intensity (A.U.)"), col='blue')
  par(new=T)
  plot(elapsedtimefvflash[1:length(error)], error, xlim=c(min(elapsedtime, na.rm=T), max(elapsedtime, na.rm=T)),
       axes=F, type='p', xlab=NA, ylab=NA, col="green", ylim=c(0, 100))
  rect(elapsedtime[stim[1]], 0, elapsedtime[stim[2]], 100, col=rgb(red=0.2, green=0.2, blue=0.2, alpha=0.2), border = NA)
  axis(4)
  mtext(paste("Tracking error (px)"), side = 4, line=3)
  legend("topright", inset=.05, c("Fluorescence", "Error"), col=c("blue",  "green"), lty = 1)
  title(main=paste0("Exposure: ", exposure, " ms, ", "Binning: ", binning, ", ", "fps: ", fpsfl))
  dev.off()
  
  # Plot speed and tracking error
  png(file=paste0(dir, sample, "_speed_error_", type, ".png"), width=800, height=600)
  par(mar=c(5,4,3,4))
  plot(elapsedtimefvflash[1:length(error)], error, xlim=c(min(elapsedtime, na.rm=T), max(elapsedtime, na.rm=T)),
       type='p', xlab=NA, ylab="Tracking error (px)", col="green", ylim=c(0, 100))
  par(new=T)
  plot(elapsedtimefvflash[1:length(speed)], speed, xlim=c(min(elapsedtime, na.rm=T), max(elapsedtime, na.rm=T)),
       axes=F, type='p', xlab=NA, ylab=NA, col="red", ylim=c(0, 60))
  rect(elapsedtime[stim[1]], 0, elapsedtime[stim[2]], 60, col=rgb(red=0.2, green=0.2, blue=0.2, alpha=0.2), border = NA)
  axis(4)
  mtext(paste("Fly speed (mm/s)"), side = 4, line=3)
  legend("topright", inset=.05, c("Speed", "Error"), col=c("red", "green"), lty = 1)
  title(main=paste0("Exposure: ", exposure, " ms, ", "Binning: ", binning, ", ", "fps: ", fpsfl))
  dev.off()
  
  # Plot speed, fluorescence, and window size
  png(file=paste0(dir, sample, "_speed_intensity_window_", type, ".png"), width=800, height=600)
  par(mar=c(5,4,3,4))
  plot(elapsedtime, objsize, xlim=c(min(elapsedtime, na.rm=T), max(elapsedtime, na.rm=T)),
       axes=F, type='l', xlab=NA, ylab=NA, col="green", ylim=c(0, median(objsize, na.rm=T)*1.5))
  par(new=T)
  plot(elapsedtime, intensity, type="l", xlim=c(min(elapsedtime, na.rm=T), max(elapsedtime, na.rm=T)), 
       ylim=c(0, median(intensity, na.rm=T)*3), xlab=paste("Elapsed time (ms) from", starttimefl), 
       ylab=paste("Fluorescence intensity (A.U.)"), col='blue')
  par(new=T)
  plot(elapsedtimefvflash[1:length(speed)], speed, xlim=c(min(elapsedtime, na.rm=T), max(elapsedtime, na.rm=T)),
       axes=F, type='p', xlab=NA, ylab=NA, col="red", ylim=c(0, 60))
  rect(elapsedtime[stim[1]], 0, elapsedtime[stim[2]], 60, col=rgb(red=0.2, green=0.2, blue=0.2, alpha=0.2), border = NA)
  axis(4)
  mtext(paste("Fly speed (mm/s), Window size (px)"), side = 4, line=3)
  legend("topright", inset=.05, c("Fluorescence","Speed", "Window size"), col=c("blue", "red", "green"), lty = 1)
  title(main=paste0("Exposure: ", exposure, " ms, ", "Binning: ", binning, ", ", "fps: ", fpsfl))
  dev.off()
  
  # Plot speed and window size
  png(file=paste0(dir, sample, "_speed_window_", type, ".png"), width=800, height=600)
  par(mar=c(5,4,3,4))
  plot(elapsedtimefvflash[1:length(speed)], speed, xlim=c(min(elapsedtime, na.rm=T), max(elapsedtime, na.rm=T)),
       type='l', xlab=NA, ylab="Fly speed (mm/s)", col="red", ylim=c(0, 60))
  rect(elapsedtime[stim[1]], 0, elapsedtime[stim[2]], 60, col=rgb(red=0.2, green=0.2, blue=0.2, alpha=0.2), border = NA)
  par(new=T)
  plot(elapsedtime, objsize, xlim=c(min(elapsedtime, na.rm=T), max(elapsedtime, na.rm=T)),
       axes=F, type='l', xlab=NA, ylab=NA, col="green", ylim=c(0, median(objsize, na.rm=T)*1.5))
  axis(4)
  mtext(paste("Window size (px)"), side = 4, line=3)
  legend("topright", inset=.05, c("Speed", "Window size"), col=c("red", "green"), lty = 1)
  title(main=paste0("Exposure: ", exposure, " ms, ", "Binning: ", binning, ", ", "fps: ", fpsfl))
  dev.off()
  
  # Plot fluorescence and window size
  png(file=paste0(dir, sample, "_intensity_window_", type, ".png"), width=800, height=600)
  par(mar=c(5,4,3,4))
  plot(elapsedtime, intensity, type="l", xlim=c(min(elapsedtime, na.rm=T), max(elapsedtime, na.rm=T)), 
       ylim=c(0, median(intensity, na.rm=T)*3), xlab=paste("Elapsed time (ms) from", starttimefl), 
       ylab=paste("Fluorescence intensity (A.U.)"), col='blue')
  par(new=T)
  plot(elapsedtime, objsize, xlim=c(min(elapsedtime, na.rm=T), max(elapsedtime, na.rm=T)),
       axes=F, type='l', xlab=NA, ylab=NA, col="green", ylim=c(0, median(objsize, na.rm=T)*1.5))
  rect(elapsedtime[stim[1]], 0, elapsedtime[stim[2]], median(objsize, na.rm=T)*1.5, col=rgb(red=0.2, green=0.2, blue=0.2, alpha=0.2), border = NA)
  axis(4)
  mtext(paste("Window size (px)"), side = 4, line=3)
  legend("topright", inset=.05, c("Fluorescence", "Window size"), col=c("blue", "green"), lty = 1)
  title(main=paste0("Exposure: ", exposure, " ms, ", "Binning: ", binning, ", ", "fps: ", fpsfl))
  dev.off()
  
}
