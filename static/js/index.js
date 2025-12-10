// Mobile menu toggle
document.addEventListener('DOMContentLoaded', function() {
  const burger = document.querySelector('.navbar-burger');
  const menu = document.querySelector('.navbar-menu');

  if (burger) {
    burger.addEventListener('click', function() {
      menu.classList.toggle('is-active');
      burger.classList.toggle('is-active');
    });
  }

  // Sections dropdown toggle for mobile/touch: allow expanding the dropdown when menu is open
  const dropdownParent = document.querySelector('.navbar-item.has-dropdown');
  const navbarLink = document.querySelector('.navbar-item.has-dropdown .navbar-link');
  if (dropdownParent && navbarLink) {
    navbarLink.addEventListener('click', function(e) {
      // if mobile (menu open) or on small screens, toggle dropdown
      const isMobile = window.matchMedia('(max-width: 768px)').matches || (menu && menu.classList.contains('is-active'));
      if (isMobile) {
        e.preventDefault();
        dropdownParent.classList.toggle('is-active');
      }
    });
  }

  // Smooth scrolling for navigation links
  document.querySelectorAll('.navbar-item[href^="#"]').forEach(anchor => {
    anchor.addEventListener('click', function(e) {
      e.preventDefault();
      const target = document.querySelector(this.getAttribute('href'));
      if (target) {
        target.scrollIntoView({
          behavior: 'smooth',
          block: 'start'
        });
        // Close mobile menu if open
        if (menu.classList.contains('is-active')) {
          menu.classList.remove('is-active');
          burger.classList.remove('is-active');
        }
      }
    });
  });
});

// Toggle technical sections
function toggleSection(sectionId) {
  const section = document.getElementById(sectionId);
  const isActive = section.classList.contains('active');
  
  // Close all sections
  document.querySelectorAll('.tech-section').forEach(s => {
    s.classList.remove('active');
  });
  
  // Toggle the clicked section
  if (!isActive) {
    section.classList.add('active');
    // Scroll to section if it was just opened
    setTimeout(() => {
      section.scrollIntoView({
        behavior: 'smooth',
        block: 'nearest'
      });
    }, 100);
  }
}

// Copy BibTeX to clipboard
function copyBibtex() {
  const bibtexContent = document.getElementById('bibtex-content');
  const text = bibtexContent.textContent || bibtexContent.innerText;
  
  // Create a temporary textarea element
  const textarea = document.createElement('textarea');
  textarea.value = text;
  textarea.style.position = 'fixed';
  textarea.style.opacity = '0';
  document.body.appendChild(textarea);
  textarea.select();
  textarea.setSelectionRange(0, 99999); // For mobile devices
  
  try {
    document.execCommand('copy');
    // Show feedback
    const button = event.target.closest('.copy-bibtex');
    if (button) {
      const originalText = button.innerHTML;
      button.innerHTML = '<span class="icon"><i class="fas fa-check"></i></span><span>Copied!</span>';
      button.style.background = '#10b981';
      setTimeout(() => {
        button.innerHTML = originalText;
        button.style.background = '';
      }, 2000);
    }
  } catch (err) {
    console.error('Failed to copy BibTeX:', err);
    alert('Failed to copy BibTeX. Please select and copy manually.');
  }
  
  document.body.removeChild(textarea);
}

// Carousel functionality for OSMO gallery
let osmoGalleryCurrentSlide = 0;
let osmoGalleryItems = [];

function initOsmoGallery() {
  const gallery = document.getElementById('osmo-gallery');
  if (!gallery) return;
  
  osmoGalleryItems = gallery.querySelectorAll('.item');
  if (osmoGalleryItems.length === 0) return;
  
  // Show first slide
  osmoGalleryItems[0].classList.add('is-active');
  updateOsmoGalleryIndicator();
  
  // Add touch/swipe support
  let startX = 0;
  let currentX = 0;
  let isDragging = false;
  
  gallery.addEventListener('touchstart', (e) => {
    startX = e.touches[0].clientX;
    isDragging = true;
  });
  
  gallery.addEventListener('touchmove', (e) => {
    if (!isDragging) return;
    currentX = e.touches[0].clientX;
  });
  
  gallery.addEventListener('touchend', () => {
    if (!isDragging) return;
    isDragging = false;
    
    const diffX = startX - currentX;
    const threshold = 50; // Minimum swipe distance
    
    if (Math.abs(diffX) > threshold) {
      if (diffX > 0) {
        nextOsmoSlide();
      } else {
        previousOsmoSlide();
      }
    }
  });
  
  // Mouse drag support
  gallery.addEventListener('mousedown', (e) => {
    startX = e.clientX;
    isDragging = true;
    gallery.style.cursor = 'grabbing';
  });
  
  gallery.addEventListener('mousemove', (e) => {
    if (!isDragging) return;
    currentX = e.clientX;
  });
  
  gallery.addEventListener('mouseup', () => {
    if (!isDragging) return;
    isDragging = false;
    gallery.style.cursor = 'grab';
    
    const diffX = startX - currentX;
    const threshold = 50;
    
    if (Math.abs(diffX) > threshold) {
      if (diffX > 0) {
        nextOsmoSlide();
      } else {
        previousOsmoSlide();
      }
    }
  });
  
  gallery.addEventListener('mouseleave', () => {
    isDragging = false;
    gallery.style.cursor = 'grab';
  });
  
  gallery.style.cursor = 'grab';
}

function showOsmoSlide(index) {
  if (osmoGalleryItems.length === 0) return;
  
  osmoGalleryCurrentSlide = index;
  if (osmoGalleryCurrentSlide < 0) {
    osmoGalleryCurrentSlide = osmoGalleryItems.length - 1;
  }
  if (osmoGalleryCurrentSlide >= osmoGalleryItems.length) {
    osmoGalleryCurrentSlide = 0;
  }
  
  osmoGalleryItems.forEach((item, i) => {
    if (i === osmoGalleryCurrentSlide) {
      item.classList.add('is-active');
    } else {
      item.classList.remove('is-active');
    }
  });
  
  updateOsmoGalleryIndicator();
}

function nextOsmoSlide() {
  showOsmoSlide(osmoGalleryCurrentSlide + 1);
}

function previousOsmoSlide() {
  showOsmoSlide(osmoGalleryCurrentSlide - 1);
}

function updateOsmoGalleryIndicator() {
  const indicator = document.getElementById('osmo-gallery-indicator');
  if (indicator && osmoGalleryItems.length > 0) {
    indicator.textContent = `${osmoGalleryCurrentSlide + 1} / ${osmoGalleryItems.length}`;
  }
}

// Global functions for button clicks
function nextSlide(galleryId) {
  if (galleryId === 'osmo-gallery') {
    nextOsmoSlide();
  }
}

function previousSlide(galleryId) {
  if (galleryId === 'osmo-gallery') {
    previousOsmoSlide();
  }
}

// Initialize carousel if bulma-carousel is available
if (typeof bulmaCarousel !== 'undefined') {
  document.addEventListener('DOMContentLoaded', function() {
    const carousels = document.querySelectorAll('.carousel:not(.osmo-gallery)');
    carousels.forEach(carousel => {
      bulmaCarousel.attach(carousel, {
        slidesToScroll: 1,
        slidesToShow: 1,
        loop: true,
        autoplay: true,
        autoplaySpeed: 3000,
        breakpoints: [
          { changePoint: 480, slidesToShow: 1, slidesToScroll: 1 },
          { changePoint: 640, slidesToShow: 1, slidesToScroll: 1 },
          { changePoint: 768, slidesToShow: 1, slidesToScroll: 1 }
        ]
      });
    });
  });
}

// Initialize OSMO gallery on page load
document.addEventListener('DOMContentLoaded', function() {
  initOsmoGallery();
});

// Lazy load images
if ('IntersectionObserver' in window) {
  const imageObserver = new IntersectionObserver((entries, observer) => {
    entries.forEach(entry => {
      if (entry.isIntersecting) {
        const img = entry.target;
        if (img.dataset.src) {
          img.src = img.dataset.src;
          img.removeAttribute('data-src');
        }
        observer.unobserve(img);
      }
    });
  });

  document.querySelectorAll('img[data-src]').forEach(img => {
    imageObserver.observe(img);
  });
}

