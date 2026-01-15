// Translation service for English to Urdu translation

class TranslationService {
  constructor() {
    // Cache for translations to avoid repeated processing
    this.translationCache = new Map();

    // Maximum cache size to prevent memory issues
    this.maxCacheSize = 1000;

    // Basic English to Urdu translation dictionary
    this.translationDictionary = new Map([
      ['Introduction', 'تعارف'],
      ['Overview', 'جائزہ'],
      ['Content', 'مواد'],
      ['Textbook', 'کتاب درس'],
      ['Robotics', 'روبوٹکس'],
      ['Humanoid', 'ہیومنوڈ'],
      ['Module', 'ماڈیول'],
      ['Week', 'ہفتہ'],
      ['Exercise', 'ورکشاپ'],
      ['Example', 'مثال'],
      ['Concept', ' تصور'],
      ['System', 'سسٹم'],
      ['Design', 'ڈیزائن'],
      ['Development', 'ڈویلپمنٹ'],
      ['Application', 'ایپلی کیشن'],
      ['Technology', 'ٹیکنالوجی'],
      ['Learning', 'سیکھنا'],
      ['Course', 'کورس'],
      ['Study', 'مطالعہ'],
      ['Topic', 'موضوع'],
      ['Section', 'سیکشن'],
      ['Chapter', 'باب'],
      ['Page', 'صفحہ'],
      ['Navigation', 'نیویگیشن'],
      ['Home', 'ہوم'],
      ['About', 'ہمارے بارے میں'],
      ['Contact', 'رابطہ'],
      ['Help', 'مدد'],
      ['Settings', 'ترتیبات'],
      ['Search', 'تلاش'],
      ['Menu', 'مینو'],
      ['Profile', 'پروفائل'],
      ['Login', 'لاگ ان'],
      ['Logout', 'لاگ آوٹ'],
      ['Register', 'رجسٹر'],
      ['Sign Up', 'سائن اپ'],
      ['Sign In', 'سائن ان'],
      ['Welcome', 'خوش آمدید'],
      ['Hello', 'ہیلو'],
      ['Goodbye', 'الوداع'],
      ['Thank you', 'شکریہ'],
      ['Please', 'براہ کرم'],
      ['Yes', 'جی ہاں'],
      ['No', 'نہیں'],
      ['Cancel', 'منسوخ کریں'],
      ['Save', 'محفوظ کریں'],
      ['Edit', 'ترمیم'],
      ['Delete', 'حذف کریں'],
      ['Add', 'شامل کریں'],
      ['Remove', 'ہٹا دیں'],
      ['View', 'دیکھیں'],
      ['Show', 'دکھائیں'],
      ['Hide', 'چھپائیں'],
      ['More', 'مزید'],
      ['Less', 'کم'],
      ['Next', 'اگلا'],
      ['Previous', 'پچھلا'],
      ['Continue', 'جاری رکھیں'],
      ['Start', 'شروع کریں'],
      ['End', 'ختم کریں'],
      ['Begin', 'شروع کریں'],
      ['Stop', 'روکیں'],
      ['Pause', 'موقوف'],
      ['Play', 'چلائیں'],
      ['Fast', 'تیز'],
      ['Slow', 'آہستہ'],
      ['Big', 'بڑا'],
      ['Small', 'چھوٹا'],
      ['Large', 'بڑا'],
      ['Tiny', 'نازک'],
      ['Old', 'پرانا'],
      ['New', 'نیا'],
      ['Young', 'نوجوان'],
      ['Quick', 'فوری'],
      ['Easy', 'آسان'],
      ['Hard', 'مشکل'],
      ['Difficult', 'مشکل'],
      ['Simple', 'سادہ'],
      ['Complex', 'پیچیدہ'],
      ['Advanced', 'اعلی'],
      ['Basic', 'بنیادی'],
      ['Fundamental', 'بنیادی'],
      ['Essential', 'ضروری'],
      ['Important', 'اہم'],
      ['Critical', 'اہم'],
      ['Vital', 'ضروری'],
      ['Necessary', 'ضروری'],
      ['Optional', 'اختیاری'],
      ['Required', 'ضروری'],
      ['Mandatory', 'لازمی'],
      ['Voluntary', 'رضاکارانہ'],
      ['Automatic', 'خودکار'],
      ['Manual', 'دستی'],
      ['Human', 'انسان'],
      ['Machine', 'مشین'],
      ['Computer', 'کمپیوٹر'],
      ['Software', 'سافٹ ویئر'],
      ['Hardware', 'ہارڈ ویئر'],
      ['Network', 'نیٹ ورک'],
      ['Internet', 'انٹرنیٹ'],
      ['Website', 'ویب سائٹ'],
      ['Web', 'ویب'],
      ['Online', 'آن لائن'],
      ['Offline', 'آف لائن'],
      ['Server', 'سرور'],
      ['Client', 'کلائنٹ'],
      ['Database', 'ڈیٹا بیس'],
      ['Data', 'ڈیٹا'],
      ['Information', 'معلومات'],
      ['Knowledge', 'علم'],
      ['Wisdom', 'حکمت'],
      ['Intelligence', 'ذکاء'],
      ['AI', 'مصنوعی ذکاء'],
      ['Artificial Intelligence', 'مصنوعی ذکاء'],
      ['Robot', 'روبوٹ'],
      ['Programming', 'پروگرامنگ'],
      ['Code', 'کوڈ'],
      ['Algorithm', 'الگورتھم'],
      ['Function', 'فنکشن'],
      ['Variable', 'متغیر'],
      ['Class', 'کلاس'],
      ['Object', 'آبجیکٹ'],
      ['Method', 'میتھڈ'],
      ['Property', 'پراپرٹی'],
      ['Interface', 'انٹرفیس'],
      ['API', 'API'],
      ['Application Programming Interface', 'ایپلی کیشن پروگرامنگ انٹرفیس'],
      ['Library', 'لائبریری'],
      ['Framework', 'فریم ورک'],
      ['Platform', 'پلیٹ فارم'],
      ['Architecture', 'آرکیٹیکچر'],
      ['Structure', 'ساخت'],
      ['Pattern', 'پیٹرن'],
      ['Template', 'ٹیمپلیٹ'],
      ['Model', 'ماڈل'],
      ['Simulation', 'سمولیشن'],
      ['Virtual', 'مجازی'],
      ['Real', ' حقیقی'],
      ['Physical', 'فزیکل'],
      ['Digital', 'ڈیجیٹل'],
      ['Electronic', 'الیکٹرانک'],
      ['Mechanical', 'میکانیکل'],
      ['Electrical', 'الیکٹریکل'],
      ['Chemical', 'کیمیکل'],
      ['Biological', 'بائیولوجیکل'],
      ['Natural', 'قدرتی'],
      ['Synthetic', 'سنتھیٹک'],
      ['Artificial', 'مصنوعی'],
      ['Environment', 'ماحول'],
      ['World', 'دنیا'],
      ['Earth', 'زمین'],
      ['Space', 'خلا'],
      ['Time', 'وقت'],
      ['Day', 'دن'],
      ['Night', 'رات'],
      ['Morning', 'صبح'],
      ['Evening', 'شام'],
      ['Afternoon', 'دوپہر'],
      ['Today', 'آج'],
      ['Tomorrow', 'کل'],
      ['Yesterday', 'کل'],
      ['Now', 'اب'],
      ['Later', 'بعد میں'],
      ['Soon', 'جلد']
    ]);

    // Add cache management methods
    this.cacheTimestamps = new Map(); // Track when items were added to cache
  }

  // Set cache with size management
  setCache(key, value) {
    // Check if cache is at max size
    if (this.translationCache.size >= this.maxCacheSize) {
      // Remove oldest items to make space (simple FIFO approach)
      const firstKey = this.translationCache.keys().next().value;
      if (firstKey) {
        this.translationCache.delete(firstKey);
        this.cacheTimestamps.delete(firstKey);
      }
    }

    // Add new item
    this.translationCache.set(key, value);
    this.cacheTimestamps.set(key, Date.now());
  }

  // Clear expired cache entries (older than 1 hour)
  clearExpiredCache() {
    const now = Date.now();
    const expirationTime = 60 * 60 * 1000; // 1 hour in milliseconds

    for (const [key, timestamp] of this.cacheTimestamps) {
      if (now - timestamp > expirationTime) {
        this.translationCache.delete(key);
        this.cacheTimestamps.delete(key);
      }
    }
  }

  // Clear the entire cache
  clearCache() {
    this.translationCache.clear();
    this.cacheTimestamps.clear();
  }

  // Translate a single text string using backend API
  async translateText(text, targetLang = 'ur', sourceLang = 'en') {
    if (targetLang === sourceLang) return text;

    // Clear expired cache entries periodically (every 100th call to avoid overhead)
    if (this.translationCache.size % 100 === 0) {
      this.clearExpiredCache();
    }

    // Check cache first
    const cacheKey = `${sourceLang}->${targetLang}:${text}`;
    if (this.translationCache.has(cacheKey)) {
      return this.translationCache.get(cacheKey);
    }

    try {
      // Get API base URL from environment or use default
      const apiBaseUrl = process.env.REACT_APP_API_BASE_URL || process.env.API_BASE_URL || 'http://localhost:8000';

      // Call backend API for translation
      const response = await fetch(`${apiBaseUrl}/v1/translation/translate`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          text: text,
          target_lang: targetLang,
          source_lang: sourceLang
        })
      });

      if (!response.ok) {
        // If API returns an error, try fallback
        throw new Error(`Translation API error: ${response.status} ${response.statusText}`);
      }

      const data = await response.json();
      let translatedText = data.translated_text;

      // Clean up the translation to remove mixed content
      if (targetLang === 'ur') {
        // For Urdu translations, clean up any mixed English content
        translatedText = this.cleanUrduTranslation(translatedText);
      }

      // Cache the result with size management
      this.setCache(cacheKey, translatedText);
      return translatedText;
    } catch (error) {
      console.error('Translation error:', error);

      // Log the specific error for debugging
      if (error.name === 'TypeError' && error.message.includes('fetch')) {
        console.warn('Network error during translation, using fallback');
      } else if (error.message.includes('Translation API error')) {
        console.warn('API error during translation, using fallback');
      }

      // Fallback to dictionary-based translation if API fails
      try {
        return this.fallbackTranslateText(text, targetLang, sourceLang);
      } catch (fallbackError) {
        console.error('Fallback translation also failed:', fallbackError);
        // Return original text if both API and fallback fail
        return text;
      }
    }
  }

  // Clean up Urdu translation to remove mixed English content
  cleanUrduTranslation(text) {
    if (!text) return text;

    // Remove any English text in parentheses (transliterations)
    let cleaned = text.replace(/\([^)]*\)/g, '').trim();

    // Remove extra whitespace
    cleaned = cleaned.replace(/\s+/g, ' ');

    // Remove any remaining English words if possible (simple approach)
    // This is a basic implementation - in a real application, you might want more sophisticated text processing

    return cleaned;
  }

  // Fallback dictionary-based translation method
  fallbackTranslateText(text, targetLang = 'ur', sourceLang = 'en') {
    if (targetLang === sourceLang) return text;

    let translatedText = text;

    if (targetLang === 'ur') {
      // For now, use dictionary-based translation for known words
      let result = text;

      // Sort dictionary keys by length (descending) to avoid partial matches
      const sortedKeys = Array.from(this.translationDictionary.keys())
        .sort((a, b) => b.length - a.length);

      for (const englishWord of sortedKeys) {
        const urduWord = this.translationDictionary.get(englishWord);
        // Use word boundaries to avoid partial matches
        const regex = new RegExp(`\\b${englishWord}\\b`, 'gi');
        result = result.replace(regex, urduWord);
      }

      translatedText = result;
    } else if (targetLang === 'en') {
      // Reverse translation - find Urdu and convert to English
      let result = text;

      for (const [englishWord, urduWord] of this.translationDictionary) {
        const regex = new RegExp(urduWord, 'g');
        result = result.replace(regex, englishWord);
      }

      translatedText = result;
    }

    return translatedText;
  }

  // Translate multiple texts with optimization
  async translateTexts(texts, targetLang = 'ur', sourceLang = 'en') {
    // For multiple texts, we can optimize by checking cache first and making batch API calls
    // For now, we'll use the individual method but in a real system we could implement batch API calls
    const promises = texts.map(text => this.translateText(text, targetLang, sourceLang));
    return await Promise.all(promises);
  }

  // Batch translate multiple texts using backend API (more efficient for large batches)
  async batchTranslate(texts, targetLang = 'ur', sourceLang = 'en') {
    if (!texts || texts.length === 0) return [];

    // First check cache for each text
    const results = new Array(texts.length);
    const uncachedTexts = [];
    const uncachedIndices = [];

    for (let i = 0; i < texts.length; i++) {
      const cacheKey = `${sourceLang}->${targetLang}:${texts[i]}`;
      if (this.translationCache.has(cacheKey)) {
        results[i] = this.translationCache.get(cacheKey);
      } else {
        uncachedTexts.push(texts[i]);
        uncachedIndices.push(i);
      }
    }

    // If all texts were cached, return results
    if (uncachedTexts.length === 0) {
      return results;
    }

    try {
      // Get API base URL from environment or use default
      const apiBaseUrl = process.env.REACT_APP_API_BASE_URL || process.env.API_BASE_URL || 'http://localhost:8000';

      // Call backend API for batch translation
      const response = await fetch(`${apiBaseUrl}/v1/translation/batch`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(uncachedTexts.map(text => ({
          text: text,
          target_lang: targetLang,
          source_lang: sourceLang
        })))
      });

      if (!response.ok) {
        throw new Error(`Batch translation API error: ${response.status} ${response.statusText}`);
      }

      const data = await response.json();

      // Process and cache results
      for (let i = 0; i < uncachedTexts.length; i++) {
        const originalText = uncachedTexts[i];
        const translatedText = data[i]?.translated_text || this.fallbackTranslateText(originalText, targetLang, sourceLang);

        // Cache the result
        const cacheKey = `${sourceLang}->${targetLang}:${originalText}`;
        this.setCache(cacheKey, translatedText);

        // Update result at the correct index
        results[uncachedIndices[i]] = translatedText;
      }

      return results;
    } catch (error) {
      console.error('Batch translation error:', error);
      // Fallback to individual translation if batch fails
      const fallbackPromises = uncachedTexts.map(text => this.translateText(text, targetLang, sourceLang));
      const fallbackResults = await Promise.all(fallbackPromises);

      // Update results with fallback translations
      for (let i = 0; i < uncachedTexts.length; i++) {
        results[uncachedIndices[i]] = fallbackResults[i];
      }

      return results;
    }
  }

  // Get all text elements that should be translated (main content and sidebars)
  getTextElements() {
    // Target the main content area and sidebars, avoiding headers, footers, and navigation elements
    return Array.from(
      document.querySelectorAll(
        '.main-wrapper .container .markdown, ' +
        '.main-wrapper .container .col, ' +
        '.main-wrapper .container div, ' +
        '.main-wrapper .container p, ' +
        '.main-wrapper .container h1, ' +
        '.main-wrapper .container h2, ' +
        '.main-wrapper .container h3, ' +
        '.main-wrapper .container h4, ' +
        '.main-wrapper .container h5, ' +
        '.main-wrapper .container h6, ' +
        '.main-wrapper .container li, ' +
        '.main-wrapper .container td, ' +
        '.main-wrapper .container th, ' +
        '.markdown, ' +
        '.markdown p, ' +
        '.markdown h1, ' +
        '.markdown h2, ' +
        '.markdown h3, ' +
        '.markdown h4, ' +
        '.markdown h5, ' +
        '.markdown h6, ' +
        '.markdown li, ' +
        '.markdown td, ' +
        '.markdown th, ' +
        '.theme-doc-markdown, ' +
        '.theme-doc-markdown p, ' +
        '.theme-doc-markdown h1, ' +
        '.theme-doc-markdown h2, ' +
        '.theme-doc-markdown h3, ' +
        '.theme-doc-markdown h4, ' +
        '.theme-doc-markdown h5, ' +
        '.theme-doc-markdown h6, ' +
        '.theme-doc-markdown li, ' +
        '.theme-doc-markdown td, ' +
        '.theme-doc-markdown th, ' +
        '.doc-content, ' +
        '.doc-content p, ' +
        '.doc-content h1, ' +
        '.doc-content h2, ' +
        '.doc-content h3, ' +
        '.doc-content h4, ' +
        '.doc-content h5, ' +
        '.doc-content h6, ' +
        '.doc-content li, ' +
        '.doc-content td, ' +
        '.doc-content th, ' +
        // Sidebar elements
        '.sidebar, ' +
        '.sidebar .menu, ' +
        '.sidebar .menu__list, ' +
        '.sidebar .menu__list-item, ' +
        '.sidebar .menu__link, ' +
        '.sidebar .menu__label, ' +
        '.sidebar .menu__list-item-collapsible, ' +
        '.sidebar .menu__list-item-collapsible .menu__link, ' +
        '.sidebar .menu__list-item-collapsible .menu__label, ' +
        '.sidebar__title, ' +
        '.sidebar__item, ' +
        '.theme-sidebar-menu, ' +
        '.theme-sidebar-menu .menu__link, ' +
        '.theme-sidebar-menu .menu__label, ' +
        '.theme-doc-sidebar-menu, ' +
        '.theme-doc-sidebar-menu .menu__link, ' +
        '.theme-doc-sidebar-menu .menu__label, ' +
        '.menu, ' +
        '.menu__list, ' +
        '.menu__list-item, ' +
        '.menu__link, ' +
        '.menu__label, ' +
        '.menu__list-item-collapsible, ' +
        '.menu__list-item-collapsible .menu__link, ' +
        '.menu__list-item-collapsible .menu__label'
      )
    ).filter(element => {
      // Skip elements that shouldn't be translated
      return !element.classList.contains('translate-ignore') &&
             !element.closest('.navbar') &&
             !element.closest('.header') &&
             !element.closest('.navbar__brand') &&
             !element.closest('.footer__link') &&
             !element.classList.contains('navbar') &&
             !element.classList.contains('header') &&
             element.tagName !== 'BUTTON' &&
             element.tagName !== 'INPUT' &&
             element.tagName !== 'TEXTAREA' &&
             element.tagName !== 'SCRIPT' &&
             element.tagName !== 'STYLE' &&
             element.tagName !== 'CODE' &&
             element.tagName !== 'PRE' &&
             element.textContent.trim() !== '' &&
             !element.closest('.translate-ignore');
    });
  }

  // Translate page content
  async translatePageContent(targetLang = 'ur', sourceLang = 'en') {
    // Get all text elements that should be translated
    const textElements = this.getTextElements();

    // Extract all original text content
    const originalTexts = [];
    const elementsToTranslate = [];

    for (const element of textElements) {
      // Store original text as data attribute if not already stored
      if (!element.hasAttribute('data-original-text')) {
        element.setAttribute('data-original-text', element.textContent);
      }

      const originalText = element.getAttribute('data-original-text');
      originalTexts.push(originalText);
      elementsToTranslate.push(element);
    }

    // Translate all texts in batch for better performance
    const translatedTexts = await this.batchTranslate(originalTexts, targetLang, sourceLang);

    // Update all elements with translated text
    for (let i = 0; i < elementsToTranslate.length; i++) {
      const element = elementsToTranslate[i];
      const originalText = originalTexts[i];
      const translatedText = translatedTexts[i];

      // Only update if we have a valid translation and it's different from original
      if (translatedText && translatedText !== originalText) {
        element.textContent = translatedText;
        element.setAttribute('data-translated', 'true');
      }
    }

    // Update language attribute but keep body direction LTR to maintain layout
    document.documentElement.lang = targetLang;
    document.body.dir = 'ltr'; // Keep layout LTR, only content text direction changes
    document.body.classList.toggle('urdu-translation', targetLang === 'ur');

    // Add accessibility attributes for screen readers
    document.documentElement.setAttribute('lang', targetLang);
    if (targetLang === 'ur') {
      // Add Urdu-specific accessibility features
      document.body.setAttribute('aria-roledescription', 'Urdu translation');
      // Set appropriate reading direction for assistive technologies
      document.body.setAttribute('data-translation-status', 'active');
    } else {
      document.body.removeAttribute('aria-roledescription');
      document.body.removeAttribute('data-translation-status');
    }
  }

  // Revert translation to original language
  revertTranslation() {
    const translatedElements = document.querySelectorAll('[data-original-text][data-translated]');
    translatedElements.forEach(element => {
      element.textContent = element.getAttribute('data-original-text');
      element.removeAttribute('data-translated');
      element.removeAttribute('data-original-text');
    });

    // Reset language and direction
    document.documentElement.lang = 'en';
    document.documentElement.removeAttribute('lang');
    document.body.dir = 'ltr';
    document.body.classList.remove('urdu-translation');

    // Remove accessibility attributes
    document.body.removeAttribute('aria-roledescription');
    document.body.removeAttribute('data-translation-status');
  }
}

// Export a singleton instance
export const translationService = new TranslationService();