import contentPersonalizationService from './contentPersonalizationService';

describe('ContentPersonalizationService', () => {
  test('should personalize content based on user preferences', () => {
    const userPreferences = {
      has_physical_robot: true,
      has_laptop: false,
      has_mobile: false,
      personalization_enabled: true
    };

    const content = {
      content: 'This is content for physical robots'
    };

    const result = contentPersonalizationService.personalizeContent(content, userPreferences);

    expect(result).toBeDefined();
  });

  test('should return original content when personalization is disabled', () => {
    const userPreferences = {
      has_physical_robot: true,
      has_laptop: false,
      has_mobile: false,
      personalization_enabled: false
    };

    const originalContent = { content: 'original content' };

    const result = contentPersonalizationService.personalizeContent(originalContent, userPreferences);

    expect(result).toEqual(originalContent);
  });

  test('should get active hardware correctly', () => {
    const userPreferences = {
      has_physical_robot: true,
      has_laptop: true,
      has_mobile: false
    };

    const activeHardware = contentPersonalizationService.getActiveHardware(userPreferences);

    expect(activeHardware).toContain('physical_robot');
    expect(activeHardware).toContain('laptop');
    expect(activeHardware).not.toContain('mobile');
  });

  test('should get personalization summary', () => {
    const userPreferences = {
      has_physical_robot: true,
      has_laptop: false,
      has_mobile: true,
      has_other_hardware: 'custom',
      web_dev_experience: 'beginner',
      personalization_enabled: true
    };

    const summary = contentPersonalizationService.getPersonalizationSummary(userPreferences);

    expect(summary.enabled).toBe(true);
    expect(summary.activeHardware).toContain('physical_robot');
    expect(summary.activeHardware).toContain('mobile');
    expect(summary.experienceLevel).toBe('beginner');
  });

  test('should determine if content should be shown based on preferences', () => {
    const userPreferences = {
      has_physical_robot: true,
      personalization_enabled: true
    };

    const shouldShow = contentPersonalizationService.shouldShowContent('hardware-specific', userPreferences);

    // This should return true since user has physical robot and content type is hardware-specific
    expect(shouldShow).toBe(true);
  });

  test('should handle general content when no specific hardware selected', () => {
    const userPreferences = {
      has_physical_robot: false,
      has_laptop: false,
      has_mobile: false,
      personalization_enabled: true
    };

    const shouldShow = contentPersonalizationService.shouldShowContent('general', userPreferences);

    expect(shouldShow).toBe(true);
  });
});