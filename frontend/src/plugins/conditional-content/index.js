// @ts-check

/** @type {import('@docusaurus/types').PluginModule} */
const ConditionalContentPlugin = () => {
  return {
    name: 'docusaurus-plugin-conditional-content',
    extendCli(cli) {
      cli
        .command('generate-conditional-content')
        .description('Generate conditional content components')
        .action(() => {
          console.log('Conditional content components are available in the theme.');
        });
    },
  };
};

module.exports = ConditionalContentPlugin;