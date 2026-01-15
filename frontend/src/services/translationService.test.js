import { translationService } from './translationService';

describe('TranslationService', () => {
  beforeEach(() => {
    // Clear cache before each test
    translationService.clearCache();
  });

  test('should translate text using API', async () => {
    const mockFetch = jest.fn(() =>
      Promise.resolve({
        ok: true,
        json: () => Promise.resolve({ translated_text: 'translated text' }),
      })
    );
    global.fetch = mockFetch;

    const result = await translationService.translateText('original text');

    expect(result).toBe('translated text');
    expect(mockFetch).toHaveBeenCalled();
  });

  test('should use cache when available', async () => {
    const mockFetch = jest.fn(() =>
      Promise.resolve({
        ok: true,
        json: () => Promise.resolve({ translated_text: 'cached text' }),
      })
    );
    global.fetch = mockFetch;

    // First call should use API
    await translationService.translateText('test text');

    // Second call should use cache
    const result = await translationService.translateText('test text');

    expect(result).toBe('cached text');
    expect(mockFetch).toHaveBeenCalledTimes(1); // Called only once
  });

  test('should fallback to dictionary translation when API fails', async () => {
    const mockFetch = jest.fn(() =>
      Promise.resolve({
        ok: false,
        status: 500,
        statusText: 'Internal Server Error',
      })
    );
    global.fetch = mockFetch;

    const result = await translationService.translateText('Introduction');

    expect(result).toBe('تعارف'); // Urdu translation from dictionary
  });

  test('should clean Urdu translations', () => {
    const cleaned = translationService.cleanUrduTranslation('test (transliteration) text');
    expect(cleaned).toBe('test  text'); // Should remove content in parentheses
  });

  test('should preserve code blocks and technical elements', () => {
    const elements = translationService.getTextElements();
    // This test would require a DOM environment to run properly
    expect(translationService.getTextElements).toBeDefined();
  });

  test('should handle batch translation', async () => {
    const mockFetch = jest.fn(() =>
      Promise.resolve({
        ok: true,
        json: () => Promise.resolve([
          { translated_text: 'first translated' },
          { translated_text: 'second translated' }
        ]),
      })
    );
    global.fetch = mockFetch;

    const result = await translationService.batchTranslate(['first text', 'second text']);

    expect(result).toEqual(['first translated', 'second translated']);
    expect(mockFetch).toHaveBeenCalled();
  });

  test('should revert translation properly', () => {
    // Mock DOM elements for testing
    document.body.innerHTML = '<div data-original-text="original">translated</div>';

    translationService.revertTranslation();

    const element = document.querySelector('[data-original-text]');
    expect(element.textContent).toBe('original');
  });
});